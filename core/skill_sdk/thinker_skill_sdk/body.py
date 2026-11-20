"""身体能力抽象（C 方案：关节级为基础 + 位姿级可选）。

设计要点：
- 技能不持有控制器，能力全部经 ``StepCtx.body`` 获取；
- 所有方法 tick 友好：要么幂等，要么可查询完成态（如 ``is_moving``）；
- 位姿级方法（``move_to_pose`` / ``get_tcp_pose`` / ``navigate_to``）为**可选能力**，
  由机型 ``capabilities.yaml`` 声明是否支持。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional, Protocol, Sequence


@dataclass(frozen=True)
class Pose:
    """末端/目标位姿：位置 + 四元数（``qx, qy, qz, qw``）。

    四元数为单位四元数时表示"只约束位置"（保持机型默认姿态）；非单位四元数
    表示完整 6-DoF 位姿目标，机型需按位置 + 姿态同时求解。
    """

    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def as_tuple(self) -> tuple[float, float, float, float, float, float, float]:
        return (self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)

    def is_unit_quaternion(self, tol: float = 1e-6) -> bool:
        """是否为单位四元数（即"只控位置"）。"""
        return (abs(self.qx) < tol and abs(self.qy) < tol
                and abs(self.qz) < tol and abs(abs(self.qw) - 1.0) < tol)


class ArmAPI(Protocol):
    """手臂能力。"""

    # —— 关节级（基础）——
    def move_to_joints(self, joints: Mapping[str, float], *, duration: float = 0.0) -> None: ...
    def get_joints(self) -> Mapping[str, float]: ...
    def is_moving(self) -> bool: ...

    # —— 位姿级（可选，需 capabilities.body.arm.pose_ik）——
    def move_to_pose(self, pose: Pose, *, tcp: bool = False,
                     duration: float = 0.0) -> None:
        """移动到指定位姿（幂等，可每 tick 重复下发）。

        - ``pose.is_unit_quaternion()`` 为真：只约束位置，姿态由机型求解器给出；
        - 否则按 6-DoF 位姿求解（位置 + 姿态误差同时收敛），不可达/不收敛抛
          ``SkillError(MOTION_IK_UNREACHABLE)``；
        - ``tcp=True`` 表示 ``pose`` 是夹爪抓取中心，机型按其姿态折算腕部目标。
        """
        ...

    def get_tcp_pose(self) -> Pose:
        """当前末端位姿（含四元数），到位判据用。

        口径与 ``move_to_pose(tcp=True)`` 一致：返回夹爪抓取中心位姿，
        因此可直接与下发过的目标位姿比较位置误差与姿态角误差。
        """
        ...

    def is_reachable(self, pose: Pose, *, tcp: bool = False) -> Optional[bool]:
        """``True`` 可达 / ``False`` 必不可达 / ``None`` 不确定（交给求解器）。

        只做位置级球面预检；姿态可达性由 ``move_to_pose`` 的求解结果暴露。
        """
        ...


class GripperAPI(Protocol):
    """夹爪/灵巧手能力（0..1 归一化）。"""

    def grasp(self, *, force: float = 0.5) -> None: ...
    def release(self) -> None: ...
    def set_positions(self, positions: Sequence[float]) -> None: ...
    def get_positions(self) -> list[float]:
        """返回实际各指位置（0..1 归一化）。"""
        ...
    def is_holding(self) -> bool: ...


class WaistAPI(Protocol):
    """腰部能力。"""

    def set_height(self, height: float) -> None: ...
    def set_pose(self, *, pitch: float = 0.0, roll: float = 0.0, yaw: float = 0.0) -> None: ...
    def get_pose(self) -> tuple[float, float, float]:
        """返回实际 ``(pitch, roll, yaw)``（弧度）。"""
        ...


class HeadAPI(Protocol):
    """头部能力。"""

    def look_at(self, *, yaw: float, pitch: float) -> None: ...
    def look_down(self) -> None: ...
    def get_pose(self) -> tuple[float, float]:
        """返回实际 ``(yaw, pitch)``（弧度），用于到位查询。"""
        ...


class BaseAPI(Protocol):
    """底盘能力。"""

    def set_velocity(self, vx: float, vy: float, wz: float) -> None: ...
    def stop(self) -> None: ...
    def is_moving(self) -> bool: ...

    # —— 位姿级（可选，需 capabilities.body.base.pose）——
    def navigate_to(self, pose: Pose) -> None: ...
    def arrived(self) -> bool: ...


class BodyAPI(Protocol):
    """按能力切分（不按机型暴露）。

    左右两侧同时暴露：``arms`` / ``grippers`` 以 ``"left"`` / ``"right"`` 为键；
    ``arm`` / ``gripper`` 是配置默认侧（``configs/robot.yaml`` 的 ``hand_side``）
    的兼容别名，单臂机型只需给默认侧即可（映射里也放这一侧）。
    """

    arms: Mapping[str, ArmAPI]
    grippers: Mapping[str, GripperAPI]
    arm: ArmAPI
    gripper: GripperAPI
    waist: WaistAPI
    head: HeadAPI
    base: BaseAPI
