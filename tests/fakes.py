"""Fake 后端：无 ROS、无真机的测试替身。"""
from __future__ import annotations

from typing import Any, List, Mapping, Optional

from thinker_skill_sdk import (
    ErrorCode,
    Frame,
    Pose,
    SensorAPI,
    SkillError,
    StateAPI,
    Step,
    StepCtx,
)


class _NullLog:
    def debug(self, *a, **k):
        pass

    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


class FakeArm:
    """假手臂：位姿目标立即到位（``pose_targets`` 供断言）。"""

    def __init__(self):
        self._joints: dict = {}
        self._moving = False
        self.pose = Pose(x=0.0, y=0.0, z=0.0)
        self.pose_targets: list = []
        self.reachable: Optional[bool] = True

    def move_to_joints(self, joints: Mapping[str, float], *, duration: float = 0.0):
        self._joints.update(joints)

    def get_joints(self):
        return dict(self._joints)

    def is_moving(self):
        return self._moving

    def move_to_pose(self, pose: Pose, *, tcp: bool = False,
                     duration: float = 0.0):
        if self.reachable is False:
            raise SkillError(ErrorCode.MOTION_IK_UNREACHABLE, "FakeArm: 不可达")
        self.pose_targets.append((pose, tcp))
        self.pose = pose

    def get_tcp_pose(self):
        return self.pose

    def is_reachable(self, pose: Pose, *, tcp: bool = False):
        return self.reachable


class FakeGripper:
    def __init__(self, side: str = "right"):
        self.side = side
        self.width = 1.0
        self.holding = False
        self.positions: List[float] = []
        self.set_calls: list = []

    def grasp(self, *, force: float = 0.5):
        self.width = 0.0
        self.holding = True

    def release(self):
        self.width = 1.0
        self.holding = False

    def set_positions(self, positions):
        self.positions = [float(p) for p in positions]
        self.set_calls.append(list(self.positions))
        self.width = self.positions[0] if self.positions else 0.0

    def get_positions(self):
        return list(self.positions) or [self.width]

    def is_holding(self):
        return self.holding


class FakeWaist:
    def __init__(self):
        self.pose = (0.0, 0.0, 0.0)

    def set_height(self, height: float):
        pass

    def set_pose(self, *, pitch: float = 0.0, roll: float = 0.0, yaw: float = 0.0):
        self.pose = (pitch, roll, yaw)

    def get_pose(self):
        return self.pose


class FakeHead:
    def __init__(self):
        self.yaw = 0.0
        self.pitch = 0.0

    def look_at(self, *, yaw: float, pitch: float):
        self.yaw, self.pitch = yaw, pitch

    def look_down(self):
        self.pitch = -0.5

    def get_pose(self):
        return (self.yaw, self.pitch)


class FakeBase:
    def __init__(self):
        self.vel = (0.0, 0.0, 0.0)
        self._moving = False

    def set_velocity(self, vx: float, vy: float, wz: float):
        self.vel = (vx, vy, wz)
        self._moving = any(abs(v) > 1e-6 for v in (vx, vy, wz))

    def stop(self):
        self.vel = (0.0, 0.0, 0.0)
        self._moving = False

    def is_moving(self):
        return self._moving

    def navigate_to(self, pose: Pose):
        raise NotImplementedError("pose navigation not supported by FakeBase")

    def arrived(self):
        return True


class FakeBody:
    """满足 BodyAPI 的结构化鸭子类型（左右手臂/手部同时暴露）。"""

    def __init__(self, default_side: str = "right"):
        self.default_side = default_side
        self.arms = {"left": FakeArm(), "right": FakeArm()}
        self.grippers = {s: FakeGripper(s) for s in self.arms}
        self.arm = self.arms[default_side]
        self.gripper = self.grippers[default_side]
        self.waist = FakeWaist()
        self.head = FakeHead()
        self.base = FakeBase()


class FakeSensor(SensorAPI):
    def __init__(self, frames: Optional[list] = None):
        self._frames = list(frames or [])
        self._i = 0

    def rgbd(self, *, timeout: float = 2.0) -> Frame:
        if not self._frames:
            return Frame(color=None, depth=None, stamp=0.0)
        f = self._frames[min(self._i, len(self._frames) - 1)]
        self._i += 1
        return f

    def transform_to_base(self, pose, angles=None):
        return pose


class FakeState(StateAPI):
    def __init__(self):
        self._d: dict = {}

    def get(self, key: str, default: Any = None) -> Any:
        return self._d.get(key, default)

    def set(self, key: str, value: Any) -> None:
        self._d[key] = value


def make_ctx(
    *,
    body=None,
    sensor=None,
    state=None,
    params=None,
    cfg=None,
    clock=None,
    cancelled: bool = False,
    steps: Optional[List[Step]] = None,
    log=None,
    say=None,
    call_skill=None,
) -> StepCtx:
    """构造可注入假时钟、与 ``steps`` 共享列表的 StepCtx。"""
    steps_list: List[Step] = steps if steps is not None else []
    return StepCtx(
        skill="test",
        params=params or {},
        cfg=cfg or {},
        globals={},
        body=body or FakeBody(),
        sensor=sensor or FakeSensor(),
        state=state or FakeState(),
        log=log or _NullLog(),
        say=say or (lambda s: None),
        cancelled=cancelled,
        check_cancel=lambda: None,
        now=clock or (lambda: 0.0),
        append_steps=steps_list.extend,
        call_skill=call_skill,
    )
