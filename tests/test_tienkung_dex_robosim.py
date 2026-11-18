"""robosim 后端纯逻辑测试（无 ROS / 无 MuJoCo）。"""
import inspect

import numpy as np
import pytest

from tienkung_dex_robot.robosim import load_mapping, plant as p
from tienkung_dex_robot.robosim.facade import SimHead


def test_joint_maps():
    assert p.JOINT_NAME_BY_ID[1] == "head_yaw_joint"
    assert p.ARM_IDS[0] == 11 and p.ARM_IDS[-1] == 27
    assert p.WAIST_IDS == (31, 32, 33)
    assert p.HEAD_IDS == (1, 2)


def test_clamp():
    assert p.clamp(2.0, [-1.0, 1.0]) == 1.0
    assert p.clamp(-2.0, [-1.0, 1.0]) == -1.0


def test_transform_point_identity():
    assert np.allclose(p.quat_to_matrix(1, 0, 0, 0), np.eye(3))
    out = p.transform_point([0, 0, 1], [1, 2, 3], [1, 0, 0, 0])
    assert np.allclose(out, [1, 2, 4])


def test_rgb_bgr_and_depth():
    rgb = np.array([[[1, 2, 3]]], dtype=np.uint8)
    assert np.array_equal(p.rgb_to_bgr(rgb), np.array([[[3, 2, 1]]]))
    assert int(p.depth_to_mm(np.array([[1.5]], np.float32), "32FC1")[0, 0]) == 1500
    assert int(p.depth_to_mm(np.array([[1500]], np.uint16), "16UC1")[0, 0]) == 1500


def test_sim_head_roundtrip():
    class _Group:
        def __init__(self):
            self.target = {}

        def move_to(self, ids):
            self.target = dict(ids)

        def get_states(self):
            return {1: p.JointState(pos=self.target.get(1, 0.0)),
                    2: p.JointState(pos=self.target.get(2, 0.0))}

    class _Robot:
        head = _Group()

    head = SimHead(_Robot(), {"yaw_sign": 1.0, "pitch_sign": -1.0,
                              "pitch_limits": [-1.0, 1.0],
                              "yaw_limits": [-1.0, 1.0]})
    head.look_at(yaw=0.2, pitch=0.4)
    assert _Robot.head.target[2] == pytest.approx(-0.4)
    assert head.get_pose() == pytest.approx((0.2, 0.4))


def test_mapping_resource():
    mapping = load_mapping()
    assert mapping["topics"]["head"] == "/head"
    assert mapping["topics"]["camera"] == "/ob_camera_head"
    assert mapping["gains"]["arm"]["kp"] > 0
    assert "pitch_sign" in mapping["head"]


def test_adapter_signature_compatible():
    from tienkung_dex_robot.robosim import adapter

    params = inspect.signature(adapter.create_adapter).parameters
    for name in ("node", "backend", "hand_variant", "hand_side",
                 "camera", "urdf_path", "camera_k", "pelvis_height",
                 "ik_solver"):
        assert name in params
