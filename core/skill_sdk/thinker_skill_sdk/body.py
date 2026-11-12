"""身体能力抽象（C 方案：关节级为基础 + 位姿级可选）。

设计要点：
- 技能不持有控制器，能力全部经 ``StepCtx.body`` 获取；
- 所有方法 tick 友好：要么幂等，要么可查询完成态（如 ``is_moving``）；
- 位姿级方法（``move_to_pose`` / ``get_tcp_pose`` / ``navigate_to``）为**可选能力**，
  由机型 ``capabilities.yaml`` 声明是否支持。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Protocol, Sequence


@dataclass(frozen=True)
class Pose:
    """末端/目标位姿：位置 + 四元数。"""

    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def as_tuple(self) -> tuple[float, float, float, float, float, float, float]:
        return (self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)


class ArmAPI(Protocol):
    """手臂能力。"""

    # —— 关节级（基础）——
    def move_to_joints(self, joints: Mapping[str, float], *, duration: float = 0.0) -> None: ...
    def get_joints(self) -> Mapping[str, float]: ...
    def is_moving(self) -> bool: ...

    # —— 位姿级（可选，需 capabilities.body.arm.pose_ik）——
    def move_to_pose(self, pose: Pose, *, duration: float = 0.0) -> None: ...
    def get_tcp_pose(self) -> Pose: ...


class GripperAPI(Protocol):
    """夹爪/灵巧手能力（0..1 归一化）。"""

    def grasp(self, *, force: float = 0.5) -> None: ...
    def release(self) -> None: ...
    def set_positions(self, positions: Sequence[float]) -> None: ...
    def is_holding(self) -> bool: ...


class WaistAPI(Protocol):
    """腰部能力。"""

    def set_height(self, height: float) -> None: ...
    def set_pose(self, *, pitch: float = 0.0, roll: float = 0.0, yaw: float = 0.0) -> None: ...


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
    """按能力切分（不按机型暴露）。"""

    arm: ArmAPI
    gripper: GripperAPI
    waist: WaistAPI
    head: HeadAPI
    base: BaseAPI
