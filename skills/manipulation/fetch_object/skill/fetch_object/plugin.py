"""fetch_object：把桌面上的简单物体抓到持物位（场景编排技能）。

主链（对齐参考实现 ``kaiwu_agent/fetch_object``）：:

    低头预备 → 感知定位 → 规划抓取 → 接近 → 抓取位姿 → 合手 → 提拉 → 视觉确认 → 回收

三条实现约定：

1. **低头与定位走子技能**：``ctx.call_skill`` 调用 ``look_down_table`` /
   ``detect_object``。本技能**不声明 ``body.head``**——子技能会以自己的名字
   重新申请 head 租约，父技能若已持有 head 区域会立刻 ``PRECOND_BODY_BUSY``。
2. **不写内层循环**：每个动作步骤每 tick 幂等下发一次目标位姿，用
   "位置误差 + 姿态误差 + 连续稳定" 判完成；等待用 ``ctx.now()``，不阻塞。
3. **不用力反馈判成败**：提拉后重新检测同一物体，按 "z 抬升量 + 水平距离"
   判定，失败返回可重试的 ``MOTION_GRASP_FAILED``。
"""
from __future__ import annotations

import math
from typing import Any, Callable, Dict, Mapping, Optional

from thinker_skill_sdk import ErrorCode, Pose, Step, StepResult

from .geometry import lift_position, plan_grasp, pose_delta, taught_tcp_pose
from .objects import normalize_object, resolve_params

DEFAULTS: Dict[str, Any] = {
    # 预备
    "prepare_pitch_deg": 45.0,
    # 定位
    "probe_retries": 2,
    # 动作
    "approach_tol": 0.01,
    "rot_tol": 0.05,
    "settle_ticks": 3,
    "grasp_settle_s": 0.8,
    # 合手抓握力（inspire: 0~1000，标量或 13 通道列表；None=不下发，仅位置闭合）
    "close_force": None,
    "lift_x_delta": -0.05,
    "lift_z_delta": 0.15,
    # 成败判定
    "verify_min_lift_z": 0.05,
    "verify_max_xy_dist": 0.12,
    # 接触辅助（inspire touch_data）：视觉失败但持续接触可判定成功
    "verify_use_contact": False,
    "contact_force_threshold": 50.0,       # 0.01N 单位，50=0.5N
    "contact_proximity_threshold": 100000,  # 接近觉原始值 0~16e6
    # 位姿参数（默认值来自参考实现，真机需按手型标定）
    "z_bias": 0.02,
    "base_orientation_deg": [0.0, -90.0, 0.0],
    "yaw_map": {
        "ref_x": 0.10,
        "ref_y_abs": 0.20,
        "max_extra_yaw_deg": 42.0,
        "angle_scale_deg": 28.0,
        "tanh_gain": 1.6,
    },
    "center_comp": {"x_scale": 0.01, "y_scale": 0.015},
    "tcp_delta_x": -0.03,
    "prepare_delta_y": {"left": 0.11, "right": 0.095},
    "prepare_delta_z": 0.30,
    "grasp_delta_y": 0.07,
    "grasp_delta_z": 0.135,
    "grasp_positions": [0.80, 0.80, 0.80, 0.80, 0.75, 0.75],
    # 回收（可选）：{"right": {关节名: 目标角}, "left": {...}}
    "hold_joints": {},
}


def _pose_from(vec, quat) -> Pose:
    return Pose(x=float(vec[0]), y=float(vec[1]), z=float(vec[2]),
                qx=float(quat[0]), qy=float(quat[1]),
                qz=float(quat[2]), qw=float(quat[3]))


class FetchObject:
    """抓取技能入口（``build_steps`` 可被重入，不依赖一次性副作用）。"""

    name = "fetch_object"
    requires = ["body.arm.left", "body.arm.right",
                "body.gripper.left", "body.gripper.right"]
    postcondition = {"holding": True}
    timeout = 120.0

    # ---------- 内部工具 ----------
    @staticmethod
    def _sub(c, skill: str, payload: Mapping[str, Any]):
        """同步调用子技能；host 不支持组合时返回 ``None``。"""
        call = getattr(c, "call_skill", None)
        if call is None:
            return None
        return call(skill, dict(payload))

    @staticmethod
    def _fail_from(res, extra: Optional[dict] = None) -> StepResult:
        data = dict(getattr(res, "data", None) or {})
        if extra:
            data.update(extra)
        return StepResult(status="failed", code=getattr(res, "code", None),
                          retryable=bool(getattr(res, "retryable", False)),
                          data=data)

    @staticmethod
    def _no_compose() -> StepResult:
        return StepResult(
            status="failed", code=ErrorCode.SYS_CONFIG_ERROR,
            data={"reason": "当前 host 不支持技能组合调用"})

    @staticmethod
    def _log_info(c, msg: str) -> None:
        """``ctx.log`` 可能为 None（host 未注入 logger），做一次存在性判断。"""
        info = getattr(getattr(c, "log", None), "info", None)
        if info is not None:
            info(msg)

    @staticmethod
    def _read_contact(c, side: str, cfg: Mapping[str, Any]) -> dict:
        """读接触状态（inspire touch_data；其它机型返回 any_contact=False）。"""
        reader = getattr(c.body.grippers[side], "contact_state", None)
        if reader is None:
            return {"any_contact": False}
        try:
            return reader(
                force_threshold=float(cfg["contact_force_threshold"]),
                proximity_threshold=int(cfg["contact_proximity_threshold"]))
        except Exception:  # noqa: BLE001 - 传感器异常不阻断抓取
            return {"any_contact": False, "error": "read failed"}

    # ---------- 步骤构建 ----------
    def build_steps(self, ctx, params):
        object_name = str((params or {}).get("object_name", "bottle"))
        category = normalize_object(object_name)
        cfg = resolve_params(DEFAULTS, dict(ctx.cfg or {}), category, params)
        taught = cfg.get("taught_poses") or {}
        arm_pref = str((params or {}).get("arm", "auto")).lower()
        # 成功后的语义状态：holding_object 记录抓到的类别
        self.postcondition = {"holding": True, "holding_object": category}

        tol = float(cfg["approach_tol"])
        rot_tol = float(cfg["rot_tol"])
        settle_ticks = max(1, int(cfg["settle_ticks"]))
        st: Dict[str, Any] = {
            "obj_pose": None, "side": None, "plan": None,
            "approach": None, "grasp": None, "lift": None,
            "closed": False, "closed_at": 0.0, "lifted_z": None,
        }

        # 1) 低头预备 -------------------------------------------------------
        def prepare(c):
            res = self._sub(c, "look_down_table",
                            {"pitch_deg": float(cfg["prepare_pitch_deg"])})
            if res is None:
                return self._no_compose()
            if res.state != "succeed":
                return self._fail_from(res)
            return StepResult(status="done")

        # 2) 感知定位 -------------------------------------------------------
        def locate(c):
            res = self._sub(c, "detect_object", {"object_name": object_name})
            if res is None:
                return self._no_compose()
            if res.state != "succeed":
                return self._fail_from(res)
            pose = (getattr(res, "data", None) or {}).get("obj_pose")
            if not pose:
                pose = (c.state.get("last_detection") or {}).get("obj_pose")
            if not pose or len(pose) < 3:
                return StepResult(
                    status="failed", code=ErrorCode.PERCEPTION_POSE_INVALID,
                    data={"object_name": object_name, "reason": "检测结果缺少位置"})
            st["obj_pose"] = [float(v) for v in pose[:3]]
            return StepResult(status="done", data={"obj_pose": st["obj_pose"]})

        # 3) 规划抓取 -------------------------------------------------------
        def plan(c):
            pos = st["obj_pose"]
            side = arm_pref
            if side not in ("left", "right"):
                side = "right" if float(pos[1]) < 0.0 else "left"
            if side not in getattr(c.body, "arms", {}) or \
                    side not in getattr(c.body, "grippers", {}):
                return StepResult(
                    status="failed",
                    code=ErrorCode.PRECOND_CAPABILITY_MISSING,
                    data={"object_name": object_name, "arm": side,
                          "reason": f"缺少 {side} 侧手臂或手部"})
            st["side"] = side

            # 示教位姿优先：配置了 taught_poses 且侧别匹配时，两段轨迹直接
            # 用示教的预备/抓取位姿，不再由物体位置规划偏置。
            use_taught = bool(taught) and \
                str(taught.get("side", side)).lower() == side and \
                taught.get("pregrasp") and taught.get("grasp")
            a_quat = g_quat = None
            if use_taught:
                tcp_offset = taught.get("tcp_offset") or (0.0, 0.0, 0.0)
                a_pos, a_quat = taught_tcp_pose(taught["pregrasp"], tcp_offset)
                g_pos, g_quat = taught_tcp_pose(taught["grasp"], tcp_offset)
                # 相对化：按当前物体与示教基准的 x/y 位移平移两段目标
                # （桌面高度固定，z 保持示教值；对齐 IL-LAB shift_grasp_waypoints）
                shift = (0.0, 0.0)
                if taught.get("relative_to_object") and taught.get("apple_pose_recorded"):
                    base_pose = taught["apple_pose_recorded"]
                    shift = (float(pos[0]) - float(base_pose[0]),
                             float(pos[1]) - float(base_pose[1]))
                    a_pos = (a_pos[0] + shift[0], a_pos[1] + shift[1], a_pos[2])
                    g_pos = (g_pos[0] + shift[0], g_pos[1] + shift[1], g_pos[2])
                grasp_plan = {
                    "approach": list(a_pos), "grasp": list(g_pos),
                    "quat": g_quat, "source": "taught", "shift_xy": list(shift),
                }
                self._log_info(c, "fetch_object 使用示教位姿(相对化): arm={} "
                                  "shift_xy={} pregrasp_tcp={} grasp_tcp={}".format(
                                      side, [round(v, 3) for v in shift],
                                      [round(v, 3) for v in a_pos],
                                      [round(v, 3) for v in g_pos]))
            else:
                grasp_plan = plan_grasp(pos, side, cfg)
            st["plan"] = grasp_plan

            approach = _pose_from(grasp_plan["approach"], a_quat if use_taught
                                  else grasp_plan["quat"])
            grasp = _pose_from(grasp_plan["grasp"], g_quat if use_taught
                               else grasp_plan["quat"])
            lifted = lift_position(grasp_plan["grasp"],
                                   cfg["lift_x_delta"], cfg["lift_z_delta"])
            lift = _pose_from(lifted, g_quat if use_taught
                              else grasp_plan["quat"])
            st["approach"], st["grasp"], st["lift"] = approach, grasp, lift

            arm = c.body.arms[side]
            verdict = None
            check = getattr(arm, "is_reachable", None)
            if check is not None:
                try:
                    verdict = check(approach, tcp=True)
                except TypeError:      # 老机型没有 tcp 关键字
                    verdict = check(approach)
            if verdict is False:
                return StepResult(
                    status="failed", code=ErrorCode.MOTION_IK_UNREACHABLE,
                    data={"object_name": object_name, "arm": side,
                          "obj_pose": list(pos),
                          "reason": "接近位姿超出臂展"})
            yaw_deg = math.degrees(grasp_plan["yaw"]) \
                if "yaw" in grasp_plan else None
            self._log_info(
                c, "fetch_object 规划完成: object={} category={} arm={} "
                   "obj={} source={} yaw={}".format(
                       object_name, category, side,
                       [round(v, 3) for v in pos],
                       grasp_plan.get("source", "planned"),
                       "n/a" if yaw_deg is None else round(yaw_deg, 1)))
            return StepResult(status="done", data={
                "arm": side,
                "category": category,
                "source": grasp_plan.get("source", "planned"),
                "shift_xy": grasp_plan.get("shift_xy", [0.0, 0.0]),
                "approach_pose": approach.as_tuple(),
                "grasp_pose": grasp.as_tuple(),
            })

        # 4~6, 提拉 --------------------------------------------------------
        def _mover(key: str, name: str) -> Callable:
            stable = {"n": 0}
            hand_sent = {"v": False}

            def run(c):
                side = st["side"]
                target = st[key]
                arm = c.body.arms[side]
                # 示教模式接近段：先把示教手型下发一次（幂等）
                if key == "approach" and (st.get("plan") or {}).get("source") == "taught" \
                        and not hand_sent["v"]:
                    hand_cfg = (taught.get("pregrasp") or {}).get("pregrasp_hand") or {}
                    codes = hand_cfg.get("joint_values")
                    setter = getattr(c.body.grippers[side], "set_raw_codes", None)
                    if codes and setter is not None:
                        setter(list(codes))
                        st["pregrasp_hand_codes"] = list(codes)  # 合手时保持侧摆
                    hand_sent["v"] = True
                arm.move_to_pose(target, tcp=True)
                current = arm.get_tcp_pose()
                dpos, dang = pose_delta(current.as_tuple(), target.as_tuple())
                if dpos <= tol and dang <= rot_tol:
                    stable["n"] += 1
                    if stable["n"] >= settle_ticks:
                        return StepResult(status="done", data={
                            name: {"pos_err": round(dpos, 4),
                                   "rot_err": round(dang, 4)}})
                else:
                    stable["n"] = 0
                return StepResult(status="running")

            return run

        def close_hand(c):
            side = st["side"]
            now = float(c.now())
            base_codes = st.get("pregrasp_hand_codes")
            gripper = c.body.grippers[side]
            use_force_close = bool(cfg.get("close_force_control")) and \
                base_codes and hasattr(gripper, "step_force_close")

            if use_force_close and st.get("force_close") is None:
                # 力反馈闭环：初始化渐进闭合状态
                gripper.start_force_close(
                    list(cfg["grasp_positions"]), base_codes,
                    total_steps=int(cfg.get("close_steps", 8)))
                st["force_close"] = {"stable": 0}

            if use_force_close:
                info = gripper.step_force_close(float(cfg["close_force_target"]))
                st["force_close"].update(info)
                if info["reached"]:
                    st["force_close"]["stable"] += 1
                else:
                    st["force_close"]["stable"] = 0
                # 到力并稳定若干 tick，或走完步数 → 进入收尾
                if info["done"] and st["force_close"]["stable"] >= \
                        max(0, int(cfg.get("close_settle_ticks", 2))):
                    if not st["closed"]:
                        st["closed"] = True
                        st["closed_at"] = now
                        st["contact"] = self._read_contact(c, side, cfg)
                return StepResult(
                    status="done" if st["closed"] else "running",
                    data={"force_close": dict(st["force_close"]),
                          "contact": st.get("contact")})

            # ---- 原路径：一次性闭合 ----
            if not st["closed"]:
                # 先设抓握力（inspire），再做位置闭合；None 表示仅位置控制
                force_cfg = cfg.get("close_force")
                if force_cfg is not None and hasattr(gripper, "set_force"):
                    if isinstance(force_cfg, (list, tuple)):
                        forces = list(force_cfg)
                    else:
                        forces = [float(force_cfg)] * 13
                    gripper.set_force(forces)
                if base_codes and hasattr(gripper, "set_grip_keep_abduction"):
                    gripper.set_grip_keep_abduction(
                        list(cfg["grasp_positions"]), base_codes)
                else:
                    gripper.set_positions(list(cfg["grasp_positions"]))
                st["closed"] = True
                st["closed_at"] = now
            if now - float(st["closed_at"]) >= float(cfg["grasp_settle_s"]):
                st["contact"] = self._read_contact(c, side, cfg)
                return StepResult(status="done", data={
                    "grasp_positions": list(cfg["grasp_positions"]),
                    "abduction_kept": bool(st.get("pregrasp_hand_codes")),
                    "contact": st["contact"]})
            return StepResult(status="running")

        # 8) 视觉确认 -------------------------------------------------------
        def _contact_ok():
            if not cfg.get("verify_use_contact"):
                return False
            return bool((st.get("contact") or {}).get("any_contact"))

        def verify(c):
            before = st["obj_pose"]
            res = self._sub(c, "detect_object", {"object_name": object_name})
            if res is None:
                return self._no_compose()
            after = None
            if res.state == "succeed":
                after = (getattr(res, "data", None) or {}).get("obj_pose")
            if not after:
                if _contact_ok():
                    return StepResult(status="done", data={
                        "object_name": object_name, "category": category,
                        "arm": st["side"],
                        "obj_pose": [round(v, 4) for v in before],
                        "verified_by": "contact",
                        "contact": st.get("contact"),
                        "reason": "视觉未检出，接触判定成立"})
                return StepResult(
                    status="failed", code=ErrorCode.MOTION_GRASP_FAILED,
                    retryable=True,
                    data={"object_name": object_name, "arm": st["side"],
                          "obj_pose": list(before),
                          "reason": "提拉后未再检测到目标"})
            dz = float(after[2]) - float(before[2])
            dxy = math.hypot(float(after[0]) - float(before[0]),
                             float(after[1]) - float(before[1]))
            st["lifted_z"] = dz
            if dz < float(cfg["verify_min_lift_z"]) or \
                    dxy > float(cfg["verify_max_xy_dist"]):
                if _contact_ok():
                    return StepResult(status="done", data={
                        "object_name": object_name, "category": category,
                        "arm": st["side"],
                        "obj_pose": [round(v, 4) for v in before],
                        "lifted_z": round(dz, 4),
                        "xy_shift": round(dxy, 4),
                        "verified_by": "contact",
                        "contact": st.get("contact"),
                        "reason": "视觉判据未达标，接触判定成立"})
                return StepResult(
                    status="failed", code=ErrorCode.MOTION_GRASP_FAILED,
                    retryable=True,
                    data={"object_name": object_name, "arm": st["side"],
                          "obj_pose": list(before),
                          "lifted_z": round(dz, 4),
                          "xy_shift": round(dxy, 4),
                          "reason": "物体未随手上移"})
            return StepResult(status="done", data={
                "object_name": object_name,
                "category": category,
                "arm": st["side"],
                "obj_pose": [round(v, 4) for v in before],
                "lifted_z": round(dz, 4),
                "xy_shift": round(dxy, 4),
            })

        # 9) 回收（可选，失败降级为警告）-----------------------------------
        def hold(c):
            side = st["side"]
            arm = c.body.arms[side]
            # 优先回示教结束位姿（抓取完成后不再停在提拉位）
            taught_match = bool(taught) and \
                str(taught.get("side", side)).lower() == side
            end_cfg = taught.get("end") if taught_match else None
            if end_cfg:
                e_pos, e_quat = taught_tcp_pose(end_cfg,
                    taught.get("tcp_offset") or (0.0, 0.0, 0.0))
                target = _pose_from(e_pos, e_quat)
                arm.move_to_pose(target, tcp=True)
                current = arm.get_tcp_pose()
                dpos, dang = pose_delta(current.as_tuple(), target.as_tuple())
                if dpos <= tol and dang <= rot_tol:
                    return StepResult(status="done", data={
                        "return_to": "taught_end",
                        "pos_err": round(dpos, 4),
                        "rot_err": round(dang, 4)})
                return StepResult(status="running")
            # 回退：原 hold_joints（关节级）
            joints = (cfg.get("hold_joints") or {}).get(side) or {}
            if not joints:
                return StepResult(status="done")
            arm.move_to_joints(dict(joints))
            if arm.is_moving():
                return StepResult(status="running")
            return StepResult(status="done")

        return [
            Step(name="低头预备", kind="motion", weight=1,
                 timeout=15, run=prepare),
            Step(name="感知定位", kind="perception", weight=3,
                 tick_budget=2.0, retries=int(cfg["probe_retries"]), run=locate),
            Step(name="规划抓取", kind="io", weight=1, run=plan),
            Step(name="接近", kind="motion", weight=2, timeout=20,
                 run=_mover("approach", "approach")),
            Step(name="抓取位姿", kind="motion", weight=2, timeout=20,
                 run=_mover("grasp", "grasp")),
            Step(name="合手", kind="motion", weight=1, timeout=5, run=close_hand),
            Step(name="提拉", kind="motion", weight=2, timeout=20,
                 run=_mover("lift", "lift")),
            Step(name="视觉确认", kind="perception", weight=2,
                 tick_budget=2.0, run=verify),
            Step(name="回收", kind="motion", weight=1, timeout=15,
                 on_failure="skip", run=hold),
        ]
