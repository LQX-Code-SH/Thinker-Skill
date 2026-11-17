"""tienkung_dex 适配层单测（用 fake SDK 门面，无需真机）。"""
import math
import os
from types import SimpleNamespace

import pytest

from thinker_skill_sdk import Pose
from tienkung_dex_robot import TienkungDexBody, TienkungDexSensor

URDF = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "robots", "tienkung_dex", "resources", "urdf", "tienkung_dex_brainco.urdf")


class _Vel:
    def __init__(self, vx=0.0, vy=0.0, wz=0.0):
        self.vx, self.vy, self.wz = vx, vy, wz

    @property
    def norm(self):
        return math.sqrt(self.vx ** 2 + self.vy ** 2 + self.wz ** 2)


class _JointGroup:
    def __init__(self):
        self.calls = []
        self.states = {}

    def move_to(self, positions, spd=0.3, cur=10.0):
        self.calls.append((dict(positions), spd, cur))

    def get_states(self):
        return dict(self.states)

    def get_pose(self):
        return (self.states.get(31, SimpleNamespace(pos=0.0)).pos,
                self.states.get(32, SimpleNamespace(pos=0.0)).pos,
                self.states.get(33, SimpleNamespace(pos=0.0)).pos)

    def get_state(self, jid):
        return self.states.get(jid)


class _Walk:
    def __init__(self):
        self.velocity = _Vel()
        self.stop_norm = 0.05
        self.stopped = False

    def set_velocity(self, vx, vy, wz):
        self.velocity = _Vel(vx, vy, wz)

    def stop(self):
        self.velocity = _Vel()
        self.stopped = True


class _Hand:
    def __init__(self):
        self.positions = []
        self.status = None

    def set_positions(self, vals):
        self.positions = list(vals)

    def get_status(self):
        return self.status


class _Camera:
    def __init__(self, frame):
        self._frame = frame

    def latest(self):
        return self._frame


class _Robot:
    def __init__(self, frame=None):
        self.arm = _JointGroup()
        self.head = _JointGroup()
        self.waist = _JointGroup()
        self.walk = _Walk()
        self.hand_right = _Hand()
        self.hand_left = None
        self.cameras = {"ob_camera_head": _Camera(frame)}


def _body(frame=None, tcp_offset=None):
    return TienkungDexBody(_Robot(frame), urdf_path=URDF, pelvis_height=0.92,
                           tcp_offset=tcp_offset)


def test_arm_name_to_id_and_clamp():
    body = _body()
    body.arm.move_to_joints({"left_shoulder_pitch": 99.0, "right_elbow_pitch": -99.0})
    ids, _, _ = body.arm._robot.arm.calls[-1]
    # 限幅到 limits
    assert ids[11] == pytest.approx(2.8449)
    assert ids[24] == pytest.approx(-2.5482)


def test_arm_unknown_joint():
    body = _body()
    with pytest.raises(ValueError):
        body.arm.move_to_joints({"bogus_joint": 0.0})


def test_arm_move_to_pose_via_ik():
    body = _body()                       # hand_side=right → 右臂，y 为负
    body.arm.move_to_pose(Pose(0.30, -0.20, 0.90))
    assert body._robot.arm.calls          # 已下发关节指令


def test_arm_move_to_pose_unreachable():
    from thinker_skill_sdk import ErrorCode, SkillError

    body = _body()
    with pytest.raises(SkillError) as exc:
        body.arm.move_to_pose(Pose(1.5, 0.0, 0.9))   # 超臂展
    assert exc.value.code == ErrorCode.MOTION_IK_UNREACHABLE


# ---------- 双臂与 6-DoF 位姿 ----------

def _quat_ry(angle_rad):
    """绕 y 轴旋转的四元数（x, y, z, w）。"""
    return (0.0, math.sin(angle_rad / 2.0), 0.0, math.cos(angle_rad / 2.0))


def test_body_exposes_both_arms_and_grippers():
    body = _body()
    assert set(body.arms) == {"left", "right"}
    assert set(body.grippers) == {"left", "right"}
    # 兼容别名指向配置默认侧，且与映射同一实例
    assert body.arm is body.arms["right"]
    assert body.gripper is body.grippers["right"]
    assert body.arms["left"] is not body.arms["right"]
    assert body.arms["left"]._side == "left"


def test_arm_move_to_pose_6dof():
    """四元数非单位 → 6-DoF 位姿求解；TCP 偏置按目标姿态折算。"""
    body = _body(tcp_offset=(0.0, 0.0, -0.10))
    qx, qy, qz, qw = _quat_ry(-math.pi / 2)
    body.arm.move_to_pose(Pose(0.30, -0.20, 0.90, qx, qy, qz, qw), tcp=True)
    ids, _, _ = body._robot.arm.calls[-1]
    assert len(ids) == 7

    # 位置模式：把指令回灌成关节状态，用 FK 验证 TCP 到位
    body._robot.arm.states = {
        jid: SimpleNamespace(pos=val) for jid, val in ids.items()}
    pose = body.arm.get_tcp_pose()
    assert math.isclose(pose.x, 0.30, abs_tol=5e-3)
    assert math.isclose(pose.y, -0.20, abs_tol=5e-3)
    assert math.isclose(pose.z, 0.90, abs_tol=5e-3)
    # 姿态与目标同向（±q 等价）
    dot = abs(pose.qx * qx + pose.qy * qy + pose.qz * qz + pose.qw * qw)
    assert dot > 0.999


def test_arm_move_to_pose_6dof_unreachable_orientation():
    from thinker_skill_sdk import ErrorCode, SkillError

    body = _body()
    qx, qy, qz, qw = _quat_ry(math.pi / 2)     # 夹爪朝上：腕部不可达
    with pytest.raises(SkillError) as exc:
        body.arm.move_to_pose(Pose(0.30, -0.20, 0.90, qx, qy, qz, qw))
    assert exc.value.code == ErrorCode.MOTION_IK_UNREACHABLE


def test_get_tcp_pose_returns_unit_quaternion():
    body = _body()
    body.arm.move_to_joints({"right_shoulder_pitch": 0.0})
    body._robot.arm.states = {}
    pose = body.arm.get_tcp_pose()
    norm = math.sqrt(pose.qx ** 2 + pose.qy ** 2 + pose.qz ** 2 + pose.qw ** 2)
    assert math.isclose(norm, 1.0, abs_tol=1e-9)


def test_gripper_strict_per_side():
    """左手未装配时不能悄悄落到右手（hand_left=None）。"""
    body = _body()
    assert body.grippers["left"].is_holding() is False
    assert body.grippers["left"].get_positions() == []
    with pytest.raises(RuntimeError):
        body.grippers["left"].set_positions([0.5] * 6)


def test_arm_is_moving():
    body = _body()
    body.arm.move_to_joints({"left_shoulder_pitch": 0.0})
    assert body.arm.is_moving() is True          # 无状态 → 视为未到位
    body._robot.arm.states[11] = SimpleNamespace(pos=0.0)
    assert body.arm.is_moving() is False


def test_gripper_normalize():
    body = _body()
    body.gripper.set_positions([0.0, 0.5, 1.0])
    assert body._robot.hand_right.positions == [1, 500, 1000]


def test_gripper_grasp_release():
    body = _body()
    body.gripper.grasp()
    assert body._robot.hand_right.positions == [1000] * 6
    body.gripper.release()
    assert body._robot.hand_right.positions == [1] * 6


def test_gripper_is_holding():
    body = _body()
    body._robot.hand_right.status = SimpleNamespace(positions=(950, 960, 940, 950, 930, 900))
    assert body.gripper.is_holding() is True


def test_head_look_at_clamp():
    body = _body()
    body.head.look_at(yaw=5.0, pitch=-5.0)
    ids, _, _ = body._robot.head.calls[-1]
    assert ids[1] == pytest.approx(1.0472)
    assert ids[2] == pytest.approx(-0.2618)


def test_waist_pose_uses_higher_cur():
    body = _body()
    body.waist.set_pose(pitch=0.1, yaw=0.2)
    ids, _, cur = body._robot.waist.calls[-1]
    assert ids[31] == pytest.approx(0.1)
    assert cur == 40.0


def test_waist_height_not_supported():
    body = _body()
    with pytest.raises(NotImplementedError):
        body.waist.set_height(1.0)


def test_base_velocity_and_nav():
    body = _body()
    body.base.set_velocity(0.3, 0.0, 0.1)
    assert body.base.is_moving() is True
    body.base.stop()
    assert body.base.is_moving() is False
    with pytest.raises(NotImplementedError):
        body.base.navigate_to(Pose(0, 0, 0))


def test_sensor_rgbd():
    frame = SimpleNamespace(
        color="c", depth="d", stamp=SimpleNamespace(sec=10, nanosec=500_000_000),
        frame_id="ob_camera_head",
    )
    sensor = TienkungDexSensor(_Robot(frame))
    out = sensor.rgbd(timeout=0.1)
    assert out.color == "c"
    assert out.depth == "d"
    assert out.stamp == pytest.approx(10.5)
    assert out.frame_id == "ob_camera_head"


def test_sensor_timeout():
    sensor = TienkungDexSensor(_Robot(None), poll=0.001)
    with pytest.raises(TimeoutError):
        sensor.rgbd(timeout=0.01)


def test_waist_get_pose():
    body = _body()
    body._robot.waist.states[31] = SimpleNamespace(pos=0.1)
    body._robot.waist.states[32] = SimpleNamespace(pos=0.2)
    body._robot.waist.states[33] = SimpleNamespace(pos=0.3)
    assert body.waist.get_pose() == pytest.approx((0.1, 0.2, 0.3))


def test_gripper_get_positions():
    body = _body()
    body._robot.hand_right.status = SimpleNamespace(positions=(1, 500, 1000))
    assert body.gripper.get_positions() == pytest.approx([0.0, 0.4995, 1.0], abs=1e-3)


def test_sensor_frame_has_k():
    frame = SimpleNamespace(color="c", depth="d", stamp=None,
                            frame_id="ob_camera_head")
    sensor = TienkungDexSensor(_Robot(frame), k=[1.0, 0, 2.0, 0, 3.0, 4.0, 0, 0, 1.0])
    out = sensor.rgbd(timeout=0.1)
    assert out.k is not None and out.k[0] == 1.0
