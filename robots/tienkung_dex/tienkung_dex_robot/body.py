"""tienkung_dex BodyAPI 实现（真机可用）。

依赖 SDK ``from tienkung_dex import create_robot``；本模块延迟导入 SDK，
未安装时仍可 import 本模块，仅在创建时抛错。

映射与约定（与 SDK config/joints.yaml 一致）：
- arm   → robot.arm（关节级 + 位姿级 IK；左右两侧各一个实例）
- gripper → robot.hand_left/right（0..1 归一化 → brainco 1..1000；严格按侧取手）
- waist → robot.waist（pitch/roll/yaw）
- head  → robot.head（yaw/pitch；pitch 正值=低头）
- base  → robot.walk（速度流 20Hz，无到位反馈）
"""
from __future__ import annotations

from typing import Any, Dict, Mapping, Optional, Sequence

import numpy as np

from thinker_skill_sdk import BodyAPI, ErrorCode, Pose, SkillError

from .joints import (
    ARM_JOINTS,
    ARM_LIMITS,
    HEAD_JOINTS,
    HEAD_LIMITS,
    WAIST_JOINTS,
    WAIST_LIMITS,
    clamp,
)


class TienkungDexArm:
    """手臂：关节级 + 位姿级（IK，支持 6-DoF 姿态）。"""

    def __init__(self, robot: Any, *, side: str = "left",
                 urdf_path: Any = None, pelvis_height: float = 0.0,
                 ik_solver: str = "daqp", ik_max_iter: int = 100,
                 ik_tol: float = 1e-3, ik_rot_tol: float = 0.02,
                 ik_damping: float = 0.08,
                 tcp_offset=(0.0, 0.0, 0.0), reach_margin: float = 0.03):
        self._robot = robot
        self._target: dict = {}
        self._side = side
        self._urdf_path = urdf_path
        self._pelvis_height = pelvis_height
        self._ik_solver = ik_solver
        self._ik_max_iter = int(ik_max_iter)
        self._ik_tol = float(ik_tol)
        self._ik_rot_tol = float(ik_rot_tol)
        self._ik_damping = float(ik_damping)
        self._tcp_offset = tuple(tcp_offset or (0.0, 0.0, 0.0))
        self._reach_margin = float(reach_margin)
        self._ik = None

    def _ik_obj(self):
        if self._ik is None:
            from .ik import ArmIK
            from .kinematics import default_urdf_path
            path = self._urdf_path or default_urdf_path()
            self._ik = ArmIK(
                path, side=self._side, solver=self._ik_solver,
                tol=self._ik_tol, rot_tol=self._ik_rot_tol,
                max_iter=self._ik_max_iter,
                damping=self._ik_damping,
                base_offset=(0.0, 0.0, self._pelvis_height),
                tcp_offset=self._tcp_offset,
                reach_margin=self._reach_margin)
        return self._ik

    def _require(self):
        if getattr(self._robot, "arm", None) is None:
            raise RuntimeError("tienkung_dex: arm 子系统未启用")

    def _waist_pose(self):
        """(pitch, roll, yaw)：SDK RealJointGroup 无 get_pose，直接从状态读。"""
        from .joints import WAIST_JOINTS
        states = self._robot.waist.get_states()
        return tuple(float(states[jid].pos) if jid in states else 0.0
                     for jid in (WAIST_JOINTS["waist_pitch"],
                                 WAIST_JOINTS["waist_roll"],
                                 WAIST_JOINTS["waist_yaw"]))

    def _to_ids(self, joints: Mapping[str, float]) -> dict:
        out = {}
        for name, pos in joints.items():
            jid = ARM_JOINTS.get(name)
            if jid is None:
                raise ValueError(f"未知手臂关节: {name!r}")
            out[jid] = clamp(pos, ARM_LIMITS[jid])
        return out

    def move_to_joints(self, joints: Mapping[str, float], *, duration: float = 0.0) -> None:
        self._require()
        ids = self._to_ids(joints)
        self._robot.arm.move_to(ids)
        self._target.update(ids)

    def get_joints(self) -> Mapping[str, float]:
        self._require()
        states = self._robot.arm.get_states()
        return {name: states[jid].pos for name, jid in ARM_JOINTS.items() if jid in states}

    def is_moving(self) -> bool:
        self._require()
        if not self._target:
            return False
        states = self._robot.arm.get_states()
        for jid, tgt in self._target.items():
            s = states.get(jid)
            if s is None or abs(s.pos - tgt) > 0.02:
                return True
        return False

    def is_reachable(self, pose: Pose, *, tcp: bool = False) -> Optional[bool]:
        """球面预检；``tcp=True`` 时按末端工具偏移放宽判定。"""
        self._require()
        return self._ik_obj().is_reachable(
            [pose.x, pose.y, pose.z], tcp=tcp,
            waist=self._waist_pose())

    def move_to_pose(self, pose: Pose, *, tcp: bool = False,
                     duration: float = 0.0) -> None:
        """IK 求解后下发关节角；不可达/不收敛抛 ``SkillError``。

        单位四元数 → 只约束位置（姿态由求解器给出）；
        非单位四元数 → 6-DoF 位姿求解（位置 + 姿态同时收敛）。
        ``tcp=True`` 时 ``pose`` 为夹爪抓取中心，按 **目标姿态** 旋转折算腕部位姿。
        """
        self._require()
        from .ik import matrix_to_quat, quat_to_matrix
        from .joints import URDF_TO_BODY

        ik = self._ik_obj()
        current = self.get_joints()
        seed = [current.get(URDF_TO_BODY[n], 0.0) for n in ik.joint_names]
        waist = self._waist_pose()
        target = [pose.x, pose.y, pose.z]

        verdict = ik.is_reachable(target, tcp=tcp, waist=waist)
        if verdict is False:
            raise SkillError(
                ErrorCode.MOTION_IK_UNREACHABLE,
                f"目标超出臂展: ({pose.x:.3f},{pose.y:.3f},{pose.z:.3f})")

        if pose.is_unit_quaternion():
            wrist = target if not tcp else ik.tcp_to_wrist(target, waist)
            q, ok = ik.solve(wrist, seed, waist)
        else:
            R_tcp = quat_to_matrix((pose.qx, pose.qy, pose.qz, pose.qw))
            if tcp:
                p_wrist, R_wrist = ik.tcp_pose_to_wrist(target, R_tcp)
            else:
                p_wrist, R_wrist = target, R_tcp
            q, ok = ik.solve_pose(
                p_wrist, matrix_to_quat(R_wrist), seed, waist)
        if not ok:
            raise SkillError(
                ErrorCode.MOTION_IK_UNREACHABLE,
                f"IK 未收敛: target=({pose.x:.3f},{pose.y:.3f},{pose.z:.3f})")
        self.move_to_joints({URDF_TO_BODY[n]: float(v)
                             for n, v in zip(ik.joint_names, q)})

    def get_tcp_pose(self) -> Pose:
        """FK 返回 **TCP 抓取中心** 位姿（位置 + 四元数）。

        与 ``move_to_pose(tcp=True)`` 口径一致：腕部位置加 ``R @ tcp_offset``，
        因此可以直接和目标位姿比较做到位判据。
        """
        self._require()
        from .ik import matrix_to_quat
        from .joints import URDF_TO_BODY

        ik = self._ik_obj()
        current = self.get_joints()
        q = [current.get(URDF_TO_BODY[n], 0.0) for n in ik.joint_names]
        p, R = ik.fk_pose(q, self._waist_pose())
        tcp = p + R @ np.asarray(self._tcp_offset, dtype=float)
        qx, qy, qz, qw = matrix_to_quat(R)
        return Pose(x=float(tcp[0]), y=float(tcp[1]), z=float(tcp[2]),
                    qx=float(qx), qy=float(qy), qz=float(qz), qw=float(qw))


class TienkungDexGripper:
    """灵巧手：对外 0..1 归一化，按 ``vendor`` 分派到底层。

    - ``brainco``（保留原接口）：SDK ``robot.hand_<side>``，1..1000；
    - ``inspire``：项目侧 :class:`InspireHand`，13 关节原始角度，走
      ``angle_set`` / ``angle_actual``。

    ``inspire_hand`` 可显式注入（单测/已建好的实例）；为 None 时按 vendor 懒建。
    """

    def __init__(self, robot: Any, side: str = "right", dof: int = 6,
                 *, vendor: str = "brainco", inspire_hand: Any = None):
        self._robot = robot
        self._side = side
        self._dof = dof
        self._vendor = str(vendor).lower()
        if self._vendor not in ("brainco", "inspire"):
            raise ValueError(f"未知手部 vendor: {vendor!r}（可选 brainco/inspire）")
        self._inspire_hand = inspire_hand

    @property
    def vendor(self) -> str:
        return self._vendor

    @property
    def dof(self) -> int:
        return int(self._dof)

    @property
    def hand_format(self) -> str:
        """回放口径：``inspire13``（13 关节角）或 ``brainco6``（6 维归一化）。

        上层据此选择 ``set_joint_positions`` 还是 ``set_positions``；不要靠
        DOF 猜，brainco 也可能装配成 13 维接口。
        """
        return "inspire13" if self._vendor == "inspire" else "brainco6"

    @property
    def _hand(self):
        # 严格按侧取手：双侧装配时不能把左手请求悄悄落到右手
        return getattr(self._robot, f"hand_{self._side}", None)

    @property
    def _inspire(self):
        if self._inspire_hand is None:
            raise RuntimeError(
                "tienkung_dex: inspire 手未注入（create_adapter 应传入 node）")
        return self._inspire_hand

    def _require(self):
        if self._vendor == "brainco" and self._hand is None:
            raise RuntimeError("tienkung_dex: 灵巧手未装配（hand 子系统缺失）")

    @staticmethod
    def _to_motor(norm: float) -> int:
        norm = max(0.0, min(1.0, float(norm)))
        return max(1, min(1000, int(round(1 + 999 * norm))))

    def set_positions(self, positions: Sequence[float]) -> None:
        self._require()
        if self._vendor == "brainco":
            self._hand.set_positions([self._to_motor(p) for p in positions])
        else:
            from .inspire_hand import pico6_to_inspire13
            self._inspire.set_positions(pico6_to_inspire13(positions))

    def grasp(self, *, force: float = 0.5) -> None:
        # inspire 走 6 维 pico6 展开，brainco 走原 dof 个闭合度
        n = 6 if self._vendor == "inspire" else self._dof
        self.set_positions([1.0] * n)

    def release(self) -> None:
        n = 6 if self._vendor == "inspire" else self._dof
        self.set_positions([0.0] * n)

    def set_raw_codes(self, codes: Sequence[int]) -> None:
        """直接下发示教手型原始角码（inspire 13 码；brainco 按值域夹取）。

        供示教轨迹在接近阶段设置"抓取前手型"；调用方保证角码口径匹配机型。
        """
        self._require()
        if self._vendor == "inspire":
            self._inspire.set_positions(codes)
        else:
            self._hand.set_positions(
                [max(1, min(1000, int(round(float(v))))) for v in codes])

    def set_joint_positions(self, joints: Mapping[str, float]) -> None:
        """按 URDF 关节角（rad）下发手型（inspire 专有，13 通道）。

        键可写成 ``left_index_yaw_joint`` 或 ``index_yaw``；未给出的通道保持
        当前角码（读不到时用零位/张开）。与 :meth:`set_positions` 的 6 维
        归一化不同，这里不做 6→13 上采样，侧摆/指中关节可按文件原样回放。
        """
        self._require()
        if self._vendor != "inspire":
            raise NotImplementedError(
                "brainco 手不支持关节角回放（只有 6 维归一化 set_positions）")
        from .inspire_hand import codes_from_joint_rads
        base = None
        status = self._inspire.get_status()
        if status is not None and getattr(status, "positions", None):
            base = [int(v) for v in status.positions]
        self._inspire.set_positions(codes_from_joint_rads(joints, base))

    def get_joint_positions(self) -> Dict[str, float]:
        """当前 13 通道关节角（rad，键为 ``<side>_<short>_joint``）。

        非 inspire 手返回 ``{}``，供上层判断"能不能读起始姿态"。
        """
        if self._vendor != "inspire":
            return {}
        status = self._inspire.get_status()
        if status is None or not getattr(status, "positions", None):
            return {}
        from .inspire_hand import joint_rads_from_codes
        return joint_rads_from_codes(status.positions, side=self._side)

    def set_grip_keep_abduction(self, grip_values: Sequence[float],
                                base_codes: Sequence[int]) -> None:
        """合手：屈曲到 ``grip_values``，侧摆保持 ``base_codes``（抓取前手型）。

        inspire 用逐通道角码；brainco 无独立侧摆通道，等价于普通 set_positions。
        """
        self._require()
        if self._vendor == "inspire":
            from .inspire_hand import close_codes_keep_abduction
            self.set_raw_codes(close_codes_keep_abduction(grip_values, base_codes))
        else:
            self.set_positions(list(grip_values))

    # -- 力 / 触觉（inspire 专有；brainco 返回空、set_force 空操作）--
    def get_force_threshold(self) -> list:
        """力控阈值设定值（force_actual，单位 g，预留）；inspire 以外空。

        不是实测接触力；接触判定用 :meth:`contact_state` / :meth:`get_touch`。
        """
        if self._vendor == "inspire":
            return self._inspire.get_force_threshold()
        return []

    def get_force(self) -> list:
        """:meth:`get_force_threshold` 的兼容别名。"""
        return self.get_force_threshold()

    def get_touch(self) -> dict:
        """触觉（5 指法向/切向力 0.01N、接近觉、掌心）；inspire 以外空。"""
        if self._vendor == "inspire":
            return self._inspire.get_touch()
        return {}

    def contact_state(self, **kwargs) -> dict:
        """基于 touch_data 的接触判定；inspire 以外返回 any_contact=False。"""
        if self._vendor == "inspire":
            return self._inspire.contact_state(**kwargs)
        return {"touched": [False] * 5, "count": 0, "any_contact": False}

    def set_force(self, forces: Sequence[int]) -> None:
        """下发 13 通道力控阈值（0~1000，单位 g）；inspire 以外忽略。"""
        if self._vendor == "inspire":
            self._inspire.set_force(forces)

    def get_finger_forces(self) -> list:
        """5 指指尖法向力（0.01N 单位）；inspire 以外空。"""
        if self._vendor == "inspire":
            return list(self._inspire.get_touch().get("finger_forces", []))
        return []

    def start_force_close(self, grip_values: Sequence[float],
                          base_codes: Sequence[int], *,
                          total_steps: int = 8) -> None:
        """初始化渐进闭合状态：从 base_codes 插值到目标闭合角码。

        仅 inspire 有意义；目标闭码 = close_codes_keep_abduction(grip, base)。
        """
        if self._vendor != "inspire":
            return
        from .inspire_hand import close_codes_keep_abduction
        self._close_base = [int(v) for v in base_codes]
        self._close_target = close_codes_keep_abduction(grip_values, base_codes)
        self._close_total = max(1, int(total_steps))
        self._close_step = 0

    def step_force_close(self, target_force: float) -> dict:
        """下发下一步闭合角码并读力反馈。

        返回 ``{step, total, force_sum, reached, done}``：
        - 5 指法向力和 >= ``target_force`` → ``reached=True``，停止继续闭合；
        - 走到最后一步 → ``done=True``（含到力和到上限两种）。
        """
        if self._vendor != "inspire" or not getattr(self, "_close_target", None):
            return {"step": 0, "total": 1, "force_sum": 0.0,
                    "reached": False, "done": True}
        self._close_step = min(self._close_step + 1, self._close_total)
        t = self._close_step / float(self._close_total)
        codes = [int(round(a + (b - a) * t))
                 for a, b in zip(self._close_base, self._close_target)]
        self.set_raw_codes(codes)
        forces = self.get_finger_forces()
        fsum = float(sum(forces))
        reached = fsum >= float(target_force)
        done = reached or self._close_step >= self._close_total
        return {"step": self._close_step, "total": self._close_total,
                "force_sum": round(fsum, 1), "reached": reached, "done": done}

    def get_positions(self):
        hand = self._hand if self._vendor == "brainco" else self._inspire
        if hand is None:
            return []
        st = hand.get_status()
        if st is None or not st.positions:
            return []
        if self._vendor == "brainco":
            return [(p - 1) / 999.0 for p in st.positions]
        # inspire：角码 → rad（供上层用，13 维）
        from .inspire_hand import code_to_rad
        return [code_to_rad(i, p) for i, p in enumerate(st.positions)]

    def is_holding(self) -> bool:
        hand = self._hand if self._vendor == "brainco" else self._inspire
        if hand is None:
            return False
        st = hand.get_status()
        if st is None or not st.positions:
            return False
        if self._vendor == "brainco":
            return (sum(st.positions) / len(st.positions)) > 800
        # inspire：四个屈曲指根（通道 0/1/2/4）rad 均值 > 0.9 rad 视为抓持
        from .inspire_hand import code_to_rad
        flex = [code_to_rad(i, st.positions[i]) for i in (0, 1, 2, 4)
                if i < len(st.positions)]
        return bool(flex) and (sum(flex) / len(flex)) > 0.9


class TienkungDexWaist:
    """腰部：pitch/roll/yaw（无升降自由度）。"""

    def __init__(self, robot: Any):
        self._robot = robot

    def _require(self):
        if getattr(self._robot, "waist", None) is None:
            raise RuntimeError("tienkung_dex: waist 子系统未启用")

    def set_height(self, height: float) -> None:
        raise NotImplementedError("tienkung_dex 腰部无升降自由度，请用 set_pose")

    def set_pose(self, *, pitch: float = 0.0, roll: float = 0.0, yaw: float = 0.0) -> None:
        self._require()
        ids = {
            WAIST_JOINTS["waist_pitch"]: clamp(pitch, WAIST_LIMITS[31]),
            WAIST_JOINTS["waist_roll"]: clamp(roll, WAIST_LIMITS[32]),
            WAIST_JOINTS["waist_yaw"]: clamp(yaw, WAIST_LIMITS[33]),
        }
        # yaw 带动上半身、负载大，需更大电流（SDK 实测 cur>=40）
        self._robot.waist.move_to(ids, cur=40.0)

    def get_pose(self):
        self._require()
        states = self._robot.waist.get_states()

        def _pos(jid):
            return states[jid].pos if jid in states else 0.0

        return (
            _pos(WAIST_JOINTS["waist_pitch"]),
            _pos(WAIST_JOINTS["waist_roll"]),
            _pos(WAIST_JOINTS["waist_yaw"]),
        )


class TienkungDexHead:
    """头部：yaw / pitch（pitch 正值=低头）。"""

    def __init__(self, robot: Any):
        self._robot = robot

    def _require(self):
        if getattr(self._robot, "head", None) is None:
            raise RuntimeError("tienkung_dex: head 子系统未启用")

    def look_at(self, *, yaw: float, pitch: float) -> None:
        self._require()
        ids = {
            HEAD_JOINTS["head_yaw"]: clamp(yaw, HEAD_LIMITS[1]),
            HEAD_JOINTS["head_pitch"]: clamp(pitch, HEAD_LIMITS[2]),
        }
        self._robot.head.move_to(ids)

    def look_down(self) -> None:
        self.look_at(yaw=0.0, pitch=0.6)

    def get_pose(self):
        self._require()
        states = self._robot.head.get_states()
        jyaw = HEAD_JOINTS["head_yaw"]
        jpitch = HEAD_JOINTS["head_pitch"]
        yaw = states[jyaw].pos if jyaw in states else 0.0
        pitch = states[jpitch].pos if jpitch in states else 0.0
        return (yaw, pitch)


class TienkungDexBase:
    """底盘：矢量速度流（无位姿导航/到位反馈）。"""

    def __init__(self, robot: Any):
        self._robot = robot

    def _require(self):
        if getattr(self._robot, "walk", None) is None:
            raise RuntimeError("tienkung_dex: walk 子系统未启用")

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        self._require()
        self._robot.walk.set_velocity(vx, vy, wz)

    def stop(self) -> None:
        self._require()
        self._robot.walk.stop()

    def is_moving(self) -> bool:
        walk = getattr(self._robot, "walk", None)
        if walk is None:
            return False
        return walk.velocity.norm > walk.stop_norm

    def navigate_to(self, pose: Pose) -> None:
        raise NotImplementedError("tienkung_dex 底盘为速度流，无位姿导航")

    def arrived(self) -> bool:
        return not self.is_moving()


class TienkungDexBody(BodyAPI):
    """按能力切分的 BodyAPI 实现：左右手臂/手部同时暴露。"""

    SIDES = ("left", "right")
    model = "tienkung_dex"

    def __init__(self, robot: Any, *, hand_side: str = "right", hand_dof: int = 6,
                 hand_vendor: str = "brainco", hand_variant: str = "",
                 inspire_hands=None,
                 urdf_path: Any = None, pelvis_height: float = 0.0,
                 ik_solver: str = "daqp", ik_max_iter: int = 100,
                 ik_tol: float = 1e-3, ik_rot_tol: float = 0.02,
                 ik_damping: float = 0.08, tcp_offset=None,
                 reach_margin: float = 0.03):
        self._robot = robot
        self.default_side = hand_side
        # 只读标识：供技能做机型/末端绑定校验（不参与运动控制）
        self.hand_variant = str(hand_variant or hand_vendor)
        self.arms = {
            side: TienkungDexArm(
                robot, side=side, urdf_path=urdf_path,
                pelvis_height=pelvis_height, ik_solver=ik_solver,
                ik_max_iter=ik_max_iter, ik_tol=ik_tol,
                ik_rot_tol=ik_rot_tol, ik_damping=ik_damping,
                tcp_offset=tcp_offset, reach_margin=reach_margin)
            for side in self.SIDES}
        inspire_hands = dict(inspire_hands or {})
        self.grippers = {
            side: TienkungDexGripper(
                robot, side=side, dof=hand_dof, vendor=hand_vendor,
                inspire_hand=inspire_hands.get(side) if hand_vendor == "inspire"
                else None)
            for side in self.SIDES}
        # 兼容别名：配置默认侧
        self.arm = self.arms[hand_side]
        self.gripper = self.grippers[hand_side]
        self.waist = TienkungDexWaist(robot)
        self.head = TienkungDexHead(robot)
        self.base = TienkungDexBase(robot)

    def move_arms(self, targets: Mapping[str, Mapping[str, float]]) -> None:
        """一次下发双臂关节目标，合并成同一条 ``ArmCtrl``。

        控制桥每个周期只消费一条手臂命令：分成左右两条发布时，后一条会被
        丢弃（真机实测：同 tick 先发左臂再发右臂，右臂完全不动）。因此双臂
        动作必须合并；``targets`` 形如 ``{"left": {...}, "right": {...}}``。
        """
        ids: Dict[int, float] = {}
        for joints in targets.values():
            for name, value in (joints or {}).items():
                jid = ARM_JOINTS.get(name)
                if jid is not None:
                    ids[jid] = float(value)
        if ids:
            self._robot.arm.move_to(ids)
