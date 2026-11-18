"""动作文件（``schemaVersion: 2``）解析与关键帧采样。

文件由录制端导出，结构（参考 ``goready.json``）::

    {
      "schemaVersion": 2,
      "robot": {"name": "...", "jointNames": [...],
                "joints": {"<urdf joint>": {"lower": .., "upper": .., "velocity": ..}}},
      "activeClipId": "clip-...",
      "clips": [{
          "id": "...", "name": "...", "durationMs": 10000, "fps": 30, "loop": true,
          "playRange": {"startMs": 0, "endMs": 10000},
          "tracks": [{"type": "joint", "jointName": "...", "keyframes": [
              {"timeMs": 0, "value": 0.0, "interpolation": "auto",
               "inHandle": {"dxMs": -1000, "dy": 0.0},
               "outHandle": {"dxMs": 1000, "dy": 0.0}}]}]
      }]
    }

本模块只做纯解析与插值（不 import ROS / 机型包），便于离机单测。
关节名到 ``BodyAPI`` 的映射按天工 DEX 的 URDF 命名固定在此，见 ``ARM_URDF_TO_BODY``。
"""
from __future__ import annotations

import json
import math
import os
import re
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

SCHEMA_VERSION = 2

# URDF 关节名 → BodyAPI 手臂关节名（与 robots/tienkung_dex/joints.py 的 URDF_TO_BODY 一致）
ARM_URDF_TO_BODY: Dict[str, str] = {
    "shoulder_pitch_l_joint": "left_shoulder_pitch",
    "shoulder_roll_l_joint": "left_shoulder_roll",
    "shoulder_yaw_l_joint": "left_shoulder_yaw",
    "elbow_pitch_l_joint": "left_elbow_pitch",
    "elbow_yaw_l_joint": "left_wrist_yaw",
    "wrist_pitch_l_joint": "left_wrist_pitch",
    "wrist_roll_l_joint": "left_wrist_roll",
    "shoulder_pitch_r_joint": "right_shoulder_pitch",
    "shoulder_roll_r_joint": "right_shoulder_roll",
    "shoulder_yaw_r_joint": "right_shoulder_yaw",
    "elbow_pitch_r_joint": "right_elbow_pitch",
    "elbow_yaw_r_joint": "right_wrist_yaw",
    "wrist_pitch_r_joint": "right_wrist_pitch",
    "wrist_roll_r_joint": "right_wrist_roll",
}
ARM_BODY_TO_URDF: Dict[str, str] = {v: k for k, v in ARM_URDF_TO_BODY.items()}

# 按侧切分（下发时左/右手臂各一次 move_to_joints）
ARM_SIDE_URDF: Dict[str, Dict[str, str]] = {
    "left": {u: b for u, b in ARM_URDF_TO_BODY.items() if u.endswith("_l_joint")},
    "right": {u: b for u, b in ARM_URDF_TO_BODY.items() if u.endswith("_r_joint")},
}

# URDF 腰/头关节 → BodyAPI 元组下标（waist: pitch/roll/yaw；head: yaw/pitch）
WAIST_URDF_INDEX: Dict[str, int] = {
    "waist_pitch_joint": 0,
    "waist_roll_joint": 1,
    "waist_yaw_joint": 2,
}
HEAD_URDF_INDEX: Dict[str, int] = {"head_yaw_joint": 0, "head_pitch_joint": 1}

# 手部 6 维归一化通道（0=张开 1=闭合）→ URDF 屈曲关节名后缀。
# URDF 的 *_dip_joint 为跟随关节（mimic），不单独下发。
HAND_CHANNEL_JOINTS: Tuple[Tuple[str, str], ...] = (
    ("thumb_pitch", "thumb_mcp"),
    ("thumb_roll", "thumb_yaw"),
    ("index", "index_mcp"),
    ("middle", "middle_mcp"),
    ("ring", "ring_mcp"),
    ("little", "pinky_mcp"),
)

LEG_JOINT_RE = re.compile(r"^(hip|knee|ankle)_")


class ActionFileError(ValueError):
    """动作文件缺失 / 结构不合法 / 选择器无匹配。"""


# --------------------------------------------------------------------------- #
# 文件读取与 clip 选择
# --------------------------------------------------------------------------- #
def _finite(value: Any, what: str) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError) as exc:
        raise ActionFileError(f"{what} 不是数值: {value!r}") from exc
    if not math.isfinite(out):
        raise ActionFileError(f"{what} 不是有限数: {value!r}")
    return out


def load_action(path: str) -> Dict[str, Any]:
    """读动作文件并校验版本；失败抛 :class:`ActionFileError`。"""
    try:
        with open(path, "r", encoding="utf-8") as fh:
            doc = json.load(fh)
    except OSError as exc:
        raise ActionFileError(f"动作文件无法读取: {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ActionFileError(f"动作文件不是合法 JSON: {path}: {exc}") from exc

    if not isinstance(doc, dict):
        raise ActionFileError("动作文件根节点必须是对象")
    version = doc.get("schemaVersion")
    try:
        version_num = int(version)
    except (TypeError, ValueError) as exc:
        raise ActionFileError(f"缺少/非法的 schemaVersion: {version!r}") from exc
    if version_num != SCHEMA_VERSION:
        raise ActionFileError(
            f"不支持的动作文件版本 schemaVersion={version_num}（仅支持 {SCHEMA_VERSION}）")
    return doc


def select_clip(doc: Mapping[str, Any],
                selector: Optional[Any] = None) -> "Clip":
    """按 ``clip`` 参数（名称 / id / 下标）选择片段，缺省用 ``activeClipId``。"""
    clips = doc.get("clips") or []
    if not isinstance(clips, list) or not clips:
        raise ActionFileError("动作文件不含任何 clip")
    candidates = [c for c in clips if isinstance(c, dict)]
    if not candidates:
        raise ActionFileError("动作文件不含任何 clip")

    chosen = None
    if selector is not None and str(selector).strip() != "":
        text = str(selector).strip()
        chosen = next((c for c in candidates
                       if text in (str(c.get("name", "")), str(c.get("id", "")))), None)
        if chosen is None and text.lstrip("-").isdigit():
            idx = int(text)
            if 0 <= idx < len(candidates):
                chosen = candidates[idx]
        if chosen is None:
            names = [str(c.get("name") or c.get("id")) for c in candidates]
            raise ActionFileError(f"找不到 clip: {text!r}（可选: {names}）")
    else:
        active = doc.get("activeClipId")
        chosen = next((c for c in candidates if c.get("id") == active), None)
        if chosen is None:
            chosen = candidates[0]

    limits = ((doc.get("robot") or {}).get("joints") or {})
    if not isinstance(limits, dict):
        limits = {}
    return Clip(chosen, limits=limits)


def resolve_action_path(name: str, *, skill_dir: Optional[str] = None,
                        actions_dir: Optional[str] = None,
                        env_var: str = "THINKER_ACTIONS_DIR") -> str:
    """解析 ``action`` 参数为实际文件路径。

    顺序：绝对/相对路径本身 → ``actions_dir``（技能配置）→ ``<skill_dir>/actions`` →
    ``<skill_dir>`` → 环境变量目录 → 当前工作目录。文件名无后缀时补 ``.json``。
    """
    raw = str(name or "").strip()
    if not raw:
        raise ActionFileError("action 为空")
    names = [raw]
    if not os.path.splitext(raw)[1]:
        names.append(raw + ".json")

    direct = os.path.expanduser(raw)
    if os.path.isabs(direct):
        if os.path.isfile(direct):
            return direct
        raise ActionFileError(f"动作文件不存在: {direct}")

    dirs: List[str] = []
    if actions_dir:
        dirs.append(os.path.expanduser(str(actions_dir)))
    if skill_dir:
        dirs.append(os.path.join(skill_dir, "actions"))
        dirs.append(skill_dir)
    env_dir = os.environ.get(env_var)
    if env_dir:
        dirs.append(os.path.expanduser(env_dir))
    dirs.append(os.getcwd())

    for base in dirs:
        for item in names:
            candidate = os.path.join(base, item)
            if os.path.isfile(candidate):
                return candidate
    raise ActionFileError(
        f"动作文件不存在: {raw}（已查找: {dirs}；可用绝对路径或设置 {env_var}）")


# --------------------------------------------------------------------------- #
# 关键帧曲线
# --------------------------------------------------------------------------- #
def _handle_slope(handle: Any) -> Optional[float]:
    """Bezier 手柄 → 时间-数值斜率（value/ms）；``dxMs`` 为 0 时视为无手柄。"""
    if not isinstance(handle, Mapping):
        return None
    try:
        dx = float(handle.get("dxMs", 0.0))
        dy = float(handle.get("dy", 0.0))
    except (TypeError, ValueError):
        return None
    if abs(dx) < 1e-9 or not math.isfinite(dx) or not math.isfinite(dy):
        return None
    return dy / dx


def _smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


class Track:
    """单关节关键帧曲线：段内按 ``interpolation`` 采样（auto 走 Hermite）。"""

    __slots__ = ("name", "times", "values", "m_out", "m_in", "modes")

    def __init__(self, name: str, keyframes: Sequence[Mapping[str, Any]]):
        self.name = name
        entries: List[Tuple[float, float, str, Optional[float], Optional[float]]] = []
        for kf in keyframes or []:
            if not isinstance(kf, Mapping):
                continue
            t = _finite(kf.get("timeMs", 0.0), f"{name} keyframe.timeMs")
            v = _finite(kf.get("value", 0.0), f"{name} keyframe.value")
            mode = str(kf.get("interpolation", "auto") or "auto").lower()
            entries.append((t, v, mode,
                            _handle_slope(kf.get("inHandle")),
                            _handle_slope(kf.get("outHandle"))))
        entries.sort(key=lambda e: e[0])

        dedup: List[Tuple[float, float, str, Optional[float], Optional[float]]] = []
        for entry in entries:
            if dedup and abs(entry[0] - dedup[-1][0]) < 1e-9:
                dedup[-1] = entry          # 同一时刻重复关键帧：后者生效
            else:
                dedup.append(entry)

        self.times = [e[0] for e in dedup]
        self.values = [e[1] for e in dedup]
        self.modes = [e[2] for e in dedup]
        self.m_in = [e[3] for e in dedup]
        self.m_out = [e[4] for e in dedup]

    def __bool__(self) -> bool:
        return bool(self.times)

    @property
    def span(self) -> float:
        """关键帧数值极差（用于报告腿轨迹幅度）。"""
        return (max(self.values) - min(self.values)) if self.values else 0.0

    def value_at(self, t_ms: float) -> float:
        times = self.times
        if not times:
            return 0.0
        if t_ms <= times[0]:
            return self.values[0]
        if t_ms >= times[-1]:
            return self.values[-1]

        lo, hi = 0, len(times) - 1
        while lo < hi - 1:                 # 找到 times[lo] <= t < times[hi]
            mid = (lo + hi) // 2
            if times[mid] <= t_ms:
                lo = mid
            else:
                hi = mid

        t0, t1 = times[lo], times[hi]
        v0, v1 = self.values[lo], self.values[hi]
        mode = self.modes[lo]
        if mode in ("step", "hold"):
            return v0
        dt = t1 - t0
        if dt <= 0.0:
            return v1
        u = (t_ms - t0) / dt
        if mode == "linear":
            return v0 + (v1 - v0) * u

        secant = (v1 - v0) / dt
        m0 = self.m_out[lo] if self.m_out[lo] is not None else secant
        m1 = self.m_in[hi] if self.m_in[hi] is not None else secant
        u2, u3 = u * u, u * u * u
        return ((2 * u3 - 3 * u2 + 1) * v0
                + (u3 - 2 * u2 + u) * dt * m0
                + (-2 * u3 + 3 * u2) * v1
                + (u3 - u2) * dt * m1)


class Clip:
    """一个片段：可播放时间范围 + 关节轨道。"""

    def __init__(self, data: Mapping[str, Any], *, limits: Mapping[str, Any]):
        self.id = str(data.get("id", ""))
        self.name = str(data.get("name") or self.id or "clip")
        self.loop = bool(data.get("loop", False))
        self.fps = data.get("fps")
        self.limits: Mapping[str, Any] = limits or {}

        tracks: Dict[str, Track] = {}
        for track in data.get("tracks") or []:
            if not isinstance(track, Mapping) or track.get("type", "joint") != "joint":
                continue
            joint = track.get("jointName") or track.get("name")
            if not joint:
                continue
            parsed = Track(str(joint), track.get("keyframes") or [])
            if parsed:
                tracks[str(joint)] = parsed
        self.tracks: Dict[str, Track] = tracks

        try:
            duration = float(data.get("durationMs") or 0.0)
        except (TypeError, ValueError):
            duration = 0.0
        if duration <= 0.0 and tracks:
            duration = max(tk.times[-1] for tk in tracks.values() if tk.times)

        play = data.get("playRange") or {}
        try:
            start = float(play.get("startMs", 0.0)) if isinstance(play, Mapping) else 0.0
            end = float(play.get("endMs", duration)) if isinstance(play, Mapping) else duration
        except (TypeError, ValueError):
            start, end = 0.0, duration
        start = max(0.0, start)
        end = min(end, duration) if duration > 0.0 else end
        if not math.isfinite(start) or not math.isfinite(end) or end <= start:
            start, end = 0.0, max(duration, 0.0)
        self.start_ms = start
        self.end_ms = end
        self.duration_ms = max(0.0, duration)

    def __bool__(self) -> bool:
        return bool(self.tracks)

    @property
    def joint_names(self) -> List[str]:
        return sorted(self.tracks)

    @property
    def span_ms(self) -> float:
        return max(0.0, self.end_ms - self.start_ms)

    def sample(self, t_ms: float) -> Dict[str, float]:
        """采样所有轨道在 ``t_ms`` 的关节角（rad）。"""
        return {name: track.value_at(t_ms) for name, track in self.tracks.items()}

    def sample_hand(self, side: str, t_ms: float) -> Optional[List[Optional[float]]]:
        """采样单侧手部 6 维归一化值（0=张开 1=闭合）。

        按文件自带 ``robot.joints`` 限位归一化；无对应关节的通道返回 ``None``。
        整侧都没有手部轨道时返回 ``None``。
        """
        values: List[Optional[float]] = []
        found = False
        for _, suffix in HAND_CHANNEL_JOINTS:
            joint = f"{side}_{suffix}_joint"
            track = self.tracks.get(joint)
            if track is None:
                values.append(None)
                continue
            found = True
            raw = track.value_at(t_ms)
            low, high = self._limit(joint)
            span = high - low
            norm = 0.0 if abs(span) < 1e-9 else (raw - low) / span
            values.append(min(1.0, max(0.0, norm)))
        return values if found else None

    def _limit(self, joint: str) -> Tuple[float, float]:
        entry = self.limits.get(joint)
        if not isinstance(entry, Mapping):
            return 0.0, 1.0
        try:
            low = float(entry.get("lower", 0.0))
            high = float(entry.get("upper", 1.0))
        except (TypeError, ValueError):
            return 0.0, 1.0
        if high <= low:
            return 0.0, 1.0
        return low, high
