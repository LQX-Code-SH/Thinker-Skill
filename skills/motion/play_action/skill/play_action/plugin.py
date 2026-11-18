"""固定动作播放：按录制动作文件（``schemaVersion: 2``）在真机上回放动作。

范围与安全约定（见《决策记录》D-024）：

- 只驱动 **腰 / 头 / 双臂 / 手指**；腿部归运控（locomotion policy）所有，SDK 明确
  禁止关节级下发，因此文件里的腿轨迹一律忽略，只在返回 ``data`` 里报告幅度；
- 起步先与**机器人当前姿态**做 crossfade（默认 2s，smoothstep），避免第一帧跳变，
  且过渡结束时速度连续（不出现"先停稳再启动"）；
- 每 tick 幂等下发关节目标，播放时间分辨率受 ``tick_rate`` 限制（默认 10Hz）；
- 手型由 ``hand_mode`` 显式区分：因时 13 关节手按 URDF 关节角逐通道回放
  （``hand_format == "inspire13"``），其余手走 6 维归一化；不靠 DOF 猜口径；
- 文件里的 ``loop`` 不生效：一次调用只播一遍，需要重复由上层再次调用。

动作文件的解析与插值见 :mod:`play_action.action_file`。
"""
from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Mapping, Optional, Tuple

from thinker_skill_sdk import ErrorCode, Step, StepResult

from .action_file import (
    ARM_SIDE_URDF,
    ARM_URDF_TO_BODY,
    HAND_CHANNEL_JOINTS,
    HAND_DRIVE_JOINT_SUFFIXES,
    HEAD_URDF_INDEX,
    LEG_JOINT_RE,
    WAIST_URDF_INDEX,
    ActionFileError,
    Clip,
    load_action,
    resolve_action_path,
    select_clip,
)
from .registry import (
    ActionEntry,
    ActionRegistryError,
    load_registry,
    resolve_registry_path,
)

DEFAULTS: Dict[str, Any] = {
    "transition_s": 2.0,      # 与当前姿态衔接的过渡时长（秒，墙钟）
    "speed": 1.0,             # 播放速度倍率
    "play_hands": True,       # 是否驱动手指（无手/无轨道时自动跳过）
    "hand_mode": "auto",      # 手部回放口径：auto | 6d | 13d
    "max_transition_s": 10.0,
}

_HAND_MODES = ("auto", "6d", "13d")

# 动作文件里可驱动的关节：臂（14）+ 腰（3）+ 头（2）+ 手部通道（按 hand_mode）
_BASE_JOINTS = frozenset(
    list(ARM_URDF_TO_BODY) + list(WAIST_URDF_INDEX) + list(HEAD_URDF_INDEX))
_HAND6_JOINTS = frozenset(
    f"{side}_{suffix}_joint"
    for side in ("left", "right") for _, suffix in HAND_CHANNEL_JOINTS)
_HAND13_JOINTS = frozenset(
    f"{side}_{suffix}_joint"
    for side in ("left", "right") for suffix in HAND_DRIVE_JOINT_SUFFIXES)


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


def _supports_13d(gripper: Any) -> bool:
    """手部是否支持 13 维关节角回放（因时手显式声明 ``hand_format``）。"""
    return (getattr(gripper, "hand_format", "") == "inspire13"
            and hasattr(gripper, "set_joint_positions")
            and hasattr(gripper, "get_joint_positions"))


def _is_driven(joint: str, hand_plan: Mapping[str, str]) -> bool:
    """关节是否会被本次回放驱动（手部通道随实际 hand_plan 变化）。"""
    if joint in _BASE_JOINTS:
        return True
    for mode in hand_plan.values():
        if mode == "13d" and joint in _HAND13_JOINTS:
            return True
        if mode == "6d" and joint in _HAND6_JOINTS:
            return True
    return False


def _hand_variant(c) -> str:
    """当前末端类型：优先 ``body.hand_variant``，缺失时按 ``hand_format`` 推断。"""
    variant = str(getattr(c.body, "hand_variant", "") or "").strip().lower()
    if variant:
        return variant
    formats = {
        str(getattr(g, "hand_format", "") or "")
        for g in (getattr(c.body, "grippers", {}) or {}).values() if g is not None}
    if "inspire13" in formats:
        return "inspire"
    if "brainco6" in formats:
        return "brainco"
    return ""


def _has_capability(c, capability: str) -> bool:
    """按 BodyAPI 的鸭子类型粗查动作清单里声明的能力。"""
    cap = str(capability or "").strip()
    if not cap:
        return True
    if cap.startswith("sensor."):
        # 只读传感器由装配期 requires 把关，技能侧不重复判定
        return True
    arms = getattr(c.body, "arms", {}) or {}
    grippers = getattr(c.body, "grippers", {}) or {}
    checks = {
        "body.arm": lambda: all(arms.get(s) is not None
                                for s in ("left", "right")),
        "body.arm.left": lambda: arms.get("left") is not None,
        "body.arm.right": lambda: arms.get("right") is not None,
        "body.gripper": lambda: any(g is not None for g in grippers.values()),
        "body.gripper.left": lambda: grippers.get("left") is not None,
        "body.gripper.right": lambda: grippers.get("right") is not None,
        "body.waist": lambda: getattr(c.body, "waist", None) is not None,
        "body.head": lambda: getattr(c.body, "head", None) is not None,
        "body.base": lambda: getattr(c.body, "base", None) is not None,
    }
    check = checks.get(cap)
    return True if check is None else bool(check())


def _registry_compat(entry: ActionEntry, c, *,
                     check_hand_mode: bool = True
                     ) -> Tuple[List[str], List[str], Dict[str, Any]]:
    """校验动作清单的机型/末端/能力声明。

    返回 ``(hard, soft, compat)``：``hard`` 是明确不匹配（strict 下失败），
    ``soft`` 是无法确认（缺标识）或 ``on_mismatch: warn`` 放行的项。
    """
    hard: List[str] = []
    soft: List[str] = []
    model = str(getattr(c.body, "model", "") or "").strip()
    variant = _hand_variant(c)
    compat: Dict[str, Any] = {
        "model": model,
        "hand_variant": variant,
        "model_ok": None,
        "hand_ok": None,
        "missing": [],
    }

    if entry.models:
        if not model:
            soft.append(f"无法确认机型（动作 {entry.key} 声明适配 {list(entry.models)}）")
        elif model not in entry.models:
            hard.append(f"机型不匹配: 当前 {model}，动作 {entry.key} "
                        f"要求 {list(entry.models)}")
            compat["model_ok"] = False
        else:
            compat["model_ok"] = True

    if entry.hand_variants:
        if not variant:
            soft.append(f"无法确认末端类型（动作 {entry.key} "
                        f"声明适配 {list(entry.hand_variants)}）")
        elif variant not in entry.hand_variants:
            hard.append(f"末端类型不匹配: 当前 {variant}，动作 {entry.key} "
                        f"要求 {list(entry.hand_variants)}")
            compat["hand_ok"] = False
        else:
            compat["hand_ok"] = True

    if check_hand_mode and entry.hand_mode == "13d" \
            and variant and variant != "inspire":
        hard.append(f"动作 {entry.key} 要求 13 维手型，当前末端为 {variant}")

    missing = [cap for cap in entry.requires if not _has_capability(c, cap)]
    if missing:
        hard.append(f"缺少动作所需能力: {missing}")
        compat["missing"] = missing

    if hard and entry.on_mismatch == "warn":
        soft.extend(hard)
        hard = []
    compat["status"] = "fail" if hard else ("warn" if soft else "ok")
    return hard, soft, compat


def _looks_like_path(ref: str) -> bool:
    text = str(ref or "")
    return bool(
        os.path.isabs(text)
        or os.sep in text
        or (os.altsep is not None and os.altsep in text)
        or text.startswith((".", "~"))
        or text.lower().endswith(".json"))


class PlayAction:
    name = "play_action"
    requires = ["body.arm.left", "body.arm.right", "body.waist", "body.head"]
    postcondition: Dict[str, Any] = {}
    timeout = 300.0

    def build_steps(self, ctx, params):
        cfg = {**DEFAULTS, **dict(ctx.cfg or {})}
        params = dict(params or {})
        play_hands = bool(params.get("play_hands", cfg.get("play_hands", True)))

        # 运行期状态（闭包内共享；build_steps 可重入，每次调用独立）。
        # speed/transition/hand_mode 在 prepare 里按
        # 「调用参数 > 动作清单 > 技能配置 > 默认值」的优先级解析。
        st: Dict[str, Any] = {
            "speed": float(DEFAULTS["speed"]),
            "transition": float(DEFAULTS["transition_s"]),
            "play_hands": play_hands,
            "hand_mode_req": "auto",
            "frames": 0,
            "t0": None,
            "clip": None,          # type: Optional[Clip]
            "cur": {},             # 起始时刻的当前关节角（rad）
            "cur_hand": {},        # 起始时刻 6 维手指归一化值（能读到才填）
            "cur_joint_hands": {},  # 起始时刻 13 维手指关节角（rad）
            "hand_plan": {},       # 每侧实际口径：13d / 6d / none
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

            skill_dir = getattr(c, "skill_dir", None)
            actions_dir = cfg.get("actions_dir")
            warnings: List[str] = []

            # -- 动作清单：只做"选择与校验"，找不到时回退到原路径搜索 --------
            registry_path = resolve_registry_path(
                skill_dir=skill_dir, actions_dir=actions_dir)
            registry = None
            if registry_path:
                try:
                    registry = load_registry(registry_path)
                except ActionRegistryError as exc:
                    return StepResult(
                        status="failed", code=ErrorCode.INPUT_INVALID_PARAMS,
                        data={"error": f"动作清单非法: {exc}",
                              "registry": registry_path})
            entry: Optional[ActionEntry] = (
                registry.resolve(ref) if registry is not None else None)
            if entry is not None:
                target = entry.file
            else:
                target = ref
                if registry is not None and not _looks_like_path(ref):
                    warnings.append(
                        f"动作 {ref!r} 未在动作清单中注册，按文件路径回退")

            # -- 参数优先级：调用参数 > 动作清单 > 技能配置 > 默认值 ----------
            def _pick(name: str, entry_value: Any) -> Any:
                raw = params.get(name)
                if raw is None or (isinstance(raw, str) and not raw.strip()):
                    raw = entry_value if entry_value is not None else cfg.get(name)
                return DEFAULTS.get(name) if raw is None else raw

            speed = _clamp(float(_pick("speed", entry.speed if entry else None)),
                           0.05, 3.0)
            transition = _clamp(
                float(_pick("transition_s", entry.transition_s if entry else None)),
                0.0, float(cfg["max_transition_s"]))
            hand_mode_req = str(_pick(
                "hand_mode", entry.hand_mode if entry else None) or "auto").strip().lower()
            if hand_mode_req not in _HAND_MODES:
                return StepResult(
                    status="failed", code=ErrorCode.INPUT_INVALID_PARAMS,
                    data={"error": f"hand_mode 仅支持 {list(_HAND_MODES)}: "
                                   f"{hand_mode_req!r}",
                          "registry": registry_path,
                          "registry_key": entry.key if entry else None})

            # -- 机型/末端/能力绑定校验 ---------------------------------------
            variant = _hand_variant(c)
            hands_active = play_hands and variant != "fixed"
            compat: Dict[str, Any] = {}
            if entry is not None:
                hard, soft, compat = _registry_compat(
                    entry, c, check_hand_mode=hands_active)
                warnings.extend(soft)
                if hard:
                    return StepResult(
                        status="failed", code=ErrorCode.INPUT_INVALID_PARAMS,
                        data={"error": "动作与当前机型/末端不兼容: "
                                       + "; ".join(hard),
                              "registry": registry_path,
                              "registry_key": entry.key,
                              "compat": compat,
                              "warnings": warnings})

            try:
                path = resolve_action_path(
                    target, skill_dir=skill_dir, actions_dir=actions_dir)
                doc = load_action(path)
                selector = params.get("clip")
                if (selector is None or str(selector).strip() == "") \
                        and entry is not None:
                    selector = entry.clip or None
                clip = select_clip(doc, selector)
            except ActionFileError as exc:
                return StepResult(
                    status="failed", code=ErrorCode.INPUT_INVALID_PARAMS,
                    data={"error": str(exc),
                          "registry": registry_path,
                          "registry_key": entry.key if entry else None})

            clip_count = len(doc.get("clips") or [])
            if clip_count > 1:
                warnings.append(
                    f"动作文件含 {clip_count} 个片段，不符合“一个动作一个文件”约定"
                    f"（当前使用 {clip.name}）")

            # -- 手部口径：按末端能力选 13d/6d；显式 13d 不满足时直接失败 ----
            grippers = getattr(c.body, "grippers", {}) or {}
            sides = ([s for s in ("left", "right") if grippers.get(s) is not None]
                     if hands_active else [])
            supports13 = {s: _supports_13d(grippers[s]) for s in sides}
            if hands_active and hand_mode_req == "13d":
                if not sides:
                    return StepResult(
                        status="failed", code=ErrorCode.INPUT_INVALID_PARAMS,
                        data={"error": "当前机型/末端没有可用手部，"
                                       "无法使用 hand_mode=13d",
                              "registry_key": entry.key if entry else None})
                bad = [s for s in sides if not supports13[s]]
                if bad:
                    return StepResult(
                        status="failed", code=ErrorCode.INPUT_INVALID_PARAMS,
                        data={"error": f"手部不支持 13 维回放: {bad}（需要因时手）",
                              "registry_key": entry.key if entry else None})

            hand_plan: Dict[str, str] = {}
            for side in sides:
                if hand_mode_req == "13d":
                    mode = "13d"
                elif hand_mode_req == "6d":
                    mode = "6d"
                else:                      # auto：13 维手优先，其余走 6 维
                    mode = "13d" if supports13[side] else "6d"
                if mode == "13d" and clip.sample_hand_joints(side, clip.start_ms) is None:
                    if hand_mode_req == "13d":
                        return StepResult(
                            status="failed", code=ErrorCode.INPUT_INVALID_PARAMS,
                            data={"error": f"动作文件不含 {side} 手 13 维手部轨道",
                                  "registry_key": entry.key if entry else None})
                    mode = "6d"
                if mode == "6d" and clip.sample_hand(side, clip.start_ms) is None:
                    mode = "none"
                hand_plan[side] = mode

            if hands_active:
                mapped = [n for n in clip.joint_names if _is_driven(n, hand_plan)]
            else:
                # 手指被显式关闭/固定假手时不代表手关节"不可驱动"，只是本次
                # 不下发；保持与旧行为一致，避免只含手部轨道的动作被判无效。
                mapped = [n for n in clip.joint_names
                          if n in _BASE_JOINTS or n in _HAND6_JOINTS
                          or n in _HAND13_JOINTS]
            if not mapped:
                return StepResult(
                    status="failed", code=ErrorCode.INPUT_INVALID_PARAMS,
                    data={"error": "动作文件不含可驱动的上半身关节（仅腿/未知关节）",
                          "registry_key": entry.key if entry else None})

            leg_motion = max(
                [t.span for n, t in clip.tracks.items() if LEG_JOINT_RE.match(n)] or [0.0])
            ignored = [n for n in clip.joint_names if n not in mapped]

            if clip.robot_name and "tienkung" not in clip.robot_name.lower():
                warnings.append(
                    f"动作文件机型 {clip.robot_name} 与当前机型（tienkung_dex）不一致，"
                    "仅按同名关节回放")

            st["speed"] = speed
            st["transition"] = transition
            st["hand_mode_req"] = hand_mode_req
            st["clip"] = clip
            st["cur"] = _current_joints(c, clip)
            st["cur_hand"] = _current_hands(c)
            st["hand_plan"] = hand_plan
            st["cur_joint_hands"] = {
                side: _current_joint_hands(grippers[side])
                for side, mode in hand_plan.items() if mode == "13d"}
            st["t0"] = float(c.now())

            duration_s = clip.span_ms / 1000.0 / speed
            modes = sorted({m for m in hand_plan.values() if m != "none"})
            if not modes:
                actual = "none"
            else:
                actual = modes[0] if len(modes) == 1 else "mixed"
            return StepResult(status="done", data={
                "action": path,
                "clip": clip.name,
                "clip_id": clip.id,
                "robot_name": clip.robot_name,
                "registry": registry_path,
                "registry_key": entry.key if entry else None,
                "display_name": entry.display_name if entry else "",
                "tags": list(entry.tags) if entry else [],
                "compat": compat,
                "duration_s": round(duration_s, 3),
                "speed": speed,
                "transition_s": transition,
                "play_hands": any(m != "none" for m in hand_plan.values()),
                "hand_mode": actual,
                "hand_mode_requested": hand_mode_req,
                "hand_plan": hand_plan,
                "hand_variant": variant,
                "warnings": warnings,
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


def _current_joint_hands(gripper: Any) -> Dict[str, float]:
    """读当前 13 维手关节角（rad）；接口不可用/返回空时返回 ``{}``。"""
    reader = getattr(gripper, "get_joint_positions", None)
    if reader is None:
        return {}
    try:
        values = dict(reader() or {})
    except Exception:  # noqa: BLE001 - 读不到就退化为直接用首帧
        return {}
    out: Dict[str, float] = {}
    for name, value in values.items():
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            out[str(name)] = number
    return out


def _apply(c, st: Mapping[str, Any], pose: Mapping[str, float],
           alpha: float, t_ms: float) -> None:
    """把（已混合的）关节目标下发到身体；只命令动作文件涉及的关节。"""
    cur = st["cur"]
    blended = {name: _mix(cur.get(name), value, alpha)
               for name, value in pose.items()}

    # 双臂必须合并成一条 ArmCtrl：控制桥每个周期只消费第一条手臂命令，
    # 分侧各发一条时后一条会被丢弃（真机实测右臂完全不响应）。
    arm_targets: Dict[str, Dict[str, float]] = {}
    for side, mapping in ARM_SIDE_URDF.items():
        targets = {body_name: blended[urdf_name]
                   for urdf_name, body_name in mapping.items()
                   if urdf_name in blended}
        if targets:
            arm_targets[side] = targets
    if arm_targets:
        batch_mover = getattr(c.body, "move_arms", None)
        if batch_mover is not None:
            batch_mover(arm_targets)
        else:                              # 其它机型：退回逐侧下发
            for side, targets in arm_targets.items():
                arm = getattr(c.body, "arms", {}).get(side)
                if arm is not None:
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
    for side, mode in (st.get("hand_plan") or {}).items():
        gripper = grippers.get(side)
        if gripper is None or mode == "none":
            continue
        if mode == "13d":              # 13 维关节角：不做 6→13 上采样
            targets = clip.sample_hand_joints(side, t_ms)
            if not targets:
                continue
            cur_joints = st["cur_joint_hands"].get(side) or {}
            gripper.set_joint_positions({
                name: _mix(cur_joints.get(name), value, alpha)
                for name, value in targets.items()})
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
        gripper.set_positions(out)
