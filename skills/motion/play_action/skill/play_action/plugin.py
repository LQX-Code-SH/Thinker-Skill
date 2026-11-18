"""固定动作播放：按录制动作文件（``schemaVersion: 2``）在真机上回放动作。

范围与安全约定（见《决策记录》D-024）：

- 只驱动 **腰 / 头 / 双臂 / 手指**；腿部归运控（locomotion policy）所有，SDK 明确
  禁止关节级下发，因此文件里的腿轨迹一律忽略，只在返回 ``data`` 里报告幅度；
- 起步先与**机器人当前姿态**做 crossfade（默认 2s，smoothstep），避免第一帧跳变，
  且过渡结束时速度连续（不出现"先停稳再启动"）；
- 每 tick 幂等下发关节目标，播放时间分辨率受 ``tick_rate`` 限制（默认 10Hz）；
- 文件里的 ``loop`` 不生效：一次调用只播一遍，需要重复由上层再次调用。

动作文件的解析与插值见 :mod:`play_action.action_file`。
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Mapping, Optional

from thinker_skill_sdk import ErrorCode, Step, StepResult

from .action_file import (
    ARM_SIDE_URDF,
    ARM_URDF_TO_BODY,
    HAND_CHANNEL_JOINTS,
    HEAD_URDF_INDEX,
    LEG_JOINT_RE,
    WAIST_URDF_INDEX,
    ActionFileError,
    Clip,
    load_action,
    resolve_action_path,
    select_clip,
)

DEFAULTS: Dict[str, Any] = {
    "transition_s": 2.0,      # 与当前姿态衔接的过渡时长（秒，墙钟）
    "speed": 1.0,             # 播放速度倍率
    "play_hands": True,       # 是否驱动手指（无手/无轨道时自动跳过）
    "max_transition_s": 10.0,
}

# 动作文件里可驱动的关节：臂（14）+ 腰（3）+ 头（2）+ 手部 13 通道 ×双侧
_DRIVABLE_JOINTS = frozenset(
    list(ARM_URDF_TO_BODY) + list(WAIST_URDF_INDEX) + list(HEAD_URDF_INDEX)
    + [f"{side}_{suffix}_joint"
       for side in ("left", "right") for _, suffix in HAND_CHANNEL_JOINTS])


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _smoothstep(x: float) -> float:
    x = _clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def _mix(current: Optional[float], target: float, alpha: float) -> float:
    """当前值 → 目标值按 ``alpha`` 过渡；当前值未知时直接取目标。"""
    if current is None:
        return float(target)
    return float(current) + (float(target) - float(current)) * alpha


def _is_mapped(joint: str) -> bool:
    return joint in _DRIVABLE_JOINTS


class PlayAction:
    name = "play_action"
    requires = ["body.arm.left", "body.arm.right", "body.waist", "body.head"]
    postcondition: Dict[str, Any] = {}
    timeout = 300.0

    def build_steps(self, ctx, params):
        cfg = {**DEFAULTS, **dict(ctx.cfg or {})}
        params = dict(params or {})
        speed = _clamp(float(params.get("speed", cfg["speed"])), 0.05, 3.0)
        transition = _clamp(float(params.get("transition_s", cfg["transition_s"])),
                            0.0, float(cfg["max_transition_s"]))
        play_hands = bool(params.get("play_hands", cfg.get("play_hands", True)))

        # 运行期状态（闭包内共享；build_steps 可重入，每次调用独立）
        st: Dict[str, Any] = {
            "speed": speed,
            "transition": transition,
            "play_hands": play_hands,
            "frames": 0,
            "t0": None,
            "clip": None,          # type: Optional[Clip]
            "cur": {},             # 起始时刻的当前关节角（rad）
            "cur_hand": {},        # 起始时刻手指归一化值（能读到才填）
            "hand_ok": {"left": False, "right": False},
        }

        def _clip() -> Clip:
            clip = st["clip"]
            if clip is None:  # pragma: no cover - 由 prepare 保证
                raise ActionFileError("动作文件尚未加载")
            return clip

        # ---- 1. 读取与校验 --------------------------------------------------
        def prepare(c):
            ref = str(params.get("action") or cfg.get("action") or "").strip()
            if not ref:
                return StepResult(
                    status="failed", code=ErrorCode.INPUT_INVALID_PARAMS,
                    data={"error": "缺少 action 参数（动作文件名或路径）"})
            try:
                path = resolve_action_path(
                    ref, skill_dir=getattr(c, "skill_dir", None),
                    actions_dir=cfg.get("actions_dir"))
                doc = load_action(path)
                clip = select_clip(doc, params.get("clip"))
            except ActionFileError as exc:
                return StepResult(
                    status="failed", code=ErrorCode.INPUT_INVALID_PARAMS,
                    data={"error": str(exc)})

            mapped = [n for n in clip.joint_names if _is_mapped(n)]
            if not mapped:
                return StepResult(
                    status="failed", code=ErrorCode.INPUT_INVALID_PARAMS,
                    data={"error": "动作文件不含可驱动的上半身关节（仅腿/未知关节）"})

            leg_motion = max(
                [t.span for n, t in clip.tracks.items() if LEG_JOINT_RE.match(n)] or [0.0])
            ignored = [n for n in clip.joint_names if n not in mapped]

            st["clip"] = clip
            st["cur"] = _current_joints(c, clip)
            st["cur_hand"] = _current_hands(c)
            st["hand_ok"] = {
                side: bool(play_hands
                           and getattr(c.body, "grippers", {}).get(side) is not None
                           and hasattr(c.body.grippers[side], "set_positions")
                           and clip.sample_hand(side, clip.start_ms) is not None)
                for side in ("left", "right")}
            st["t0"] = float(c.now())

            duration_s = clip.span_ms / 1000.0 / speed
            return StepResult(status="done", data={
                "action": path,
                "clip": clip.name,
                "clip_id": clip.id,
                "duration_s": round(duration_s, 3),
                "speed": speed,
                "transition_s": transition,
                "play_hands": any(st["hand_ok"].values()),
                "ignored_joints": ignored,
                "leg_motion_rad": round(float(leg_motion), 4),
            })

        def _elapsed(c) -> float:
            return max(0.0, float(c.now()) - float(st["t0"]))

        def _sample_time(c, elapsed: float) -> float:
            clip = _clip()
            return clip.start_ms + min(clip.span_ms,
                                       elapsed * 1000.0 * float(st["speed"]))

        def _total(c) -> float:
            clip = _clip()
            return max(clip.span_ms / 1000.0 / float(st["speed"]),
                       float(st["transition"]))

        # ---- 2. 与当前姿态衔接 ----------------------------------------------
        def blend(c):
            elapsed = _elapsed(c)
            transition = float(st["transition"])
            if transition <= 0.0:
                _apply(c, st, _clip().sample(_clip().start_ms), 1.0, _clip().start_ms)
                st["frames"] += 1
                return StepResult(status="done", data={"blend_s": 0.0})

            t_ms = _sample_time(c, elapsed)
            alpha = _smoothstep(elapsed / transition)
            _apply(c, st, _clip().sample(t_ms), alpha, t_ms)
            st["frames"] += 1
            if elapsed >= transition:
                return StepResult(status="done", data={"blend_s": round(elapsed, 3)})
            return StepResult(status="running")

        # ---- 3. 播放 --------------------------------------------------------
        def play(c):
            clip = _clip()
            elapsed = _elapsed(c)
            total = _total(c)
            finished = elapsed >= total
            t_ms = clip.end_ms if finished else _sample_time(c, elapsed)
            alpha = 1.0 if finished else _smoothstep(elapsed / max(1e-6, float(st["transition"])))
            _apply(c, st, clip.sample(t_ms), alpha, t_ms)
            st["frames"] += 1
            if finished:
                return StepResult(status="done", data={
                    "played_s": round(min(elapsed, total), 3),
                    "clip_end_ms": round(clip.end_ms, 1),
                    "frames": st["frames"],
                })
            return StepResult(status="running")

        return [
            Step(name="读取动作", kind="io", weight=1, tick_budget=0.3, run=prepare),
            Step(name="姿态衔接", kind="motion", weight=1, run=blend, tick_budget=0.05),
            Step(name="播放动作", kind="motion", weight=6, run=play, tick_budget=0.05),
        ]


# --------------------------------------------------------------------------- #
# 姿态下发
# --------------------------------------------------------------------------- #
def _current_joints(c, clip: Clip) -> Dict[str, float]:
    """读起始姿态：手臂关节 + 腰 + 头（仅限动作文件里出现的关节）。"""
    cur: Dict[str, float] = {}
    wanted = set(clip.joint_names)

    for side, mapping in ARM_SIDE_URDF.items():
        arm = getattr(c.body, "arms", {}).get(side)
        if arm is None:
            continue
        try:
            actual = arm.get_joints() or {}
        except Exception:  # noqa: BLE001 - 读不到就退化为直接用首帧
            actual = {}
        for urdf_name, body_name in mapping.items():
            if urdf_name in wanted and body_name in actual:
                cur[urdf_name] = float(actual[body_name])

    # 腰/头是整体下发接口（set_pose / look_at 一次给全三个/两个量），
    # 因此只要文件里出现了其中任一关节，就把整组当前值全读出来当兜底。
    if any(n in wanted for n in WAIST_URDF_INDEX):
        try:
            values = tuple(c.body.waist.get_pose())
            for joint, idx in WAIST_URDF_INDEX.items():
                cur[joint] = float(values[idx])
        except Exception:  # noqa: BLE001
            pass

    if any(n in wanted for n in HEAD_URDF_INDEX):
        try:
            values = tuple(c.body.head.get_pose())
            for joint, idx in HEAD_URDF_INDEX.items():
                cur[joint] = float(values[idx])
        except Exception:  # noqa: BLE001
            pass

    return cur


def _current_hands(c) -> Dict[str, List[float]]:
    """读当前手指归一化位置；接口返回的不是 6 维 0..1 时视为未知。"""
    out: Dict[str, List[float]] = {}
    for side, gripper in (getattr(c.body, "grippers", {}) or {}).items():
        reader = getattr(gripper, "get_positions", None)
        if reader is None:
            continue
        try:
            values = [float(v) for v in reader() or []]
        except Exception:  # noqa: BLE001
            continue
        if len(values) == 6 and all(math.isfinite(v) and -0.01 <= v <= 1.01 for v in values):
            out[side] = values
    return out


def _apply(c, st: Mapping[str, Any], pose: Mapping[str, float],
           alpha: float, t_ms: float) -> None:
    """把（已混合的）关节目标下发到身体；只命令动作文件涉及的关节。"""
    cur = st["cur"]
    blended = {name: _mix(cur.get(name), value, alpha)
               for name, value in pose.items()}

    for side, mapping in ARM_SIDE_URDF.items():
        targets = {body_name: blended[urdf_name]
                   for urdf_name, body_name in mapping.items()
                   if urdf_name in blended}
        if not targets:
            continue
        arm = getattr(c.body, "arms", {}).get(side)
        if arm is None:
            continue
        arm.move_to_joints(targets)

    if any(name in blended for name in WAIST_URDF_INDEX):
        triple: List[Optional[float]] = [None, None, None]
        for urdf_name, idx in WAIST_URDF_INDEX.items():
            if urdf_name in blended:
                triple[idx] = blended[urdf_name]
            elif urdf_name in cur:
                triple[idx] = cur[urdf_name]
        if all(v is not None for v in triple):
            c.body.waist.set_pose(pitch=triple[0], roll=triple[1], yaw=triple[2])

    if any(name in blended for name in HEAD_URDF_INDEX):
        pair: List[Optional[float]] = [None, None]
        for urdf_name, idx in HEAD_URDF_INDEX.items():
            if urdf_name in blended:
                pair[idx] = blended[urdf_name]
            elif urdf_name in cur:
                pair[idx] = cur[urdf_name]
        if all(v is not None for v in pair):
            c.body.head.look_at(yaw=pair[0], pitch=pair[1])

    if not st["play_hands"]:
        return
    clip: Clip = st["clip"]
    grippers = getattr(c.body, "grippers", {}) or {}
    for side, enabled in st["hand_ok"].items():
        if not enabled:
            continue
        values = clip.sample_hand(side, t_ms)
        if values is None:
            continue
        current = st["cur_hand"].get(side)
        out: List[float] = []
        for i, value in enumerate(values):
            if value is None:  # 某些通道文件里没有（如 dip 跟随关节）
                value = current[i] if current else 0.0
            out.append(_clamp(_mix(current[i] if current else None, value, alpha), 0.0, 1.0))
        grippers[side].set_positions(out)
