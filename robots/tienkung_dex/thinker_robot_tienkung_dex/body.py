"""tienkung_dex BodyAPI 实现（真机可用）。

依赖 SDK ``from tienkung_dex import create_robot``；本模块延迟导入 SDK，
未安装时仍可 import 本模块，仅在创建时抛错。

映射与约定（与 SDK config/joints.yaml 一致）：
- arm   → robot.arm（关节级；位姿级/IK 不支持）
- gripper → robot.hand_left/right（0..1 归一化 → brainco 1..1000）
- waist → robot.waist（pitch/roll/yaw）
- head  → robot.head（yaw/pitch；pitch 正值=低头）
- base  → robot.walk（速度流 20Hz，无到位反馈）
"""
from __future__ import annotations

from typing import Any, Mapping, Sequence

from thinker_skill_sdk import BodyAPI, Pose

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
    """手臂：关节级（位姿级不支持）。"""

    def __init__(self, robot: Any):
        self._robot = robot
        self._target: dict = {}

    def _require(self):
        if getattr(self._robot, "arm", None) is None:
            raise RuntimeError("tienkung_dex: arm 子系统未启用")

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

    def move_to_pose(self, pose: Pose, *, duration: float = 0.0) -> None:
        raise NotImplementedError("tienkung_dex 手臂无 IK/位姿接口，请用 move_to_joints")

    def get_tcp_pose(self) -> Pose:
        raise NotImplementedError("tienkung_dex 手臂无 IK/位姿接口")


class TienkungDexGripper:
    """灵巧手：0..1 归一化 → brainco 1..1000。"""

    def __init__(self, robot: Any, side: str = "right", dof: int = 6):
        self._robot = robot
        self._side = side
        self._dof = dof

    @property
    def _hand(self):
        hand = getattr(self._robot, f"hand_{self._side}", None)
        if hand is not None:
            return hand
        return getattr(self._robot, "hand_right", None) or getattr(self._robot, "hand_left", None)

    def _require(self):
        if self._hand is None:
            raise RuntimeError("tienkung_dex: 灵巧手未装配（hand 子系统缺失）")

    @staticmethod
    def _to_motor(norm: float) -> int:
        norm = max(0.0, min(1.0, float(norm)))
        return max(1, min(1000, int(round(1 + 999 * norm))))

    def set_positions(self, positions: Sequence[float]) -> None:
        self._require()
        self._hand.set_positions([self._to_motor(p) for p in positions])

    def grasp(self, *, force: float = 0.5) -> None:
        self._require()
        self._hand.set_positions([1000] * self._dof)

    def release(self) -> None:
        self._require()
        self._hand.set_positions([1] * self._dof)

    def get_positions(self):
        hand = self._hand
        if hand is None:
            return []
        st = hand.get_status()
        if st is None or not st.positions:
            return []
        # brainco 1..1000 → 0..1
        return [(p - 1) / 999.0 for p in st.positions]

    def is_holding(self) -> bool:
        hand = self._hand
        if hand is None:
            return False
        st = hand.get_status()
        if st is not None and st.positions:
            return (sum(st.positions) / len(st.positions)) > 800
        return False


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
    """按能力切分的 BodyAPI 实现。"""

    def __init__(self, robot: Any, *, hand_side: str = "right", hand_dof: int = 6):
        self._robot = robot
        self.arm = TienkungDexArm(robot)
        self.gripper = TienkungDexGripper(robot, side=hand_side, dof=hand_dof)
        self.waist = TienkungDexWaist(robot)
        self.head = TienkungDexHead(robot)
        self.base = TienkungDexBase(robot)
