"""pelvis 高度核算测试（URDF + joints.yaml，无 ROS）。"""
import os

import pytest

from tienkung_dex_robot.leg_kinematics import (
    compute_pelvis_height,
    default_paths,
    load_pose,
)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(
    ROOT, "robots", "tienkung_dex", "resources", "urdf",
    "tienkung_dex_brainco.urdf")
JOINTS = os.path.join(
    ROOT, "robots", "tienkung_dex", "resources", "joints.yaml")


def _height(pose):
    return compute_pelvis_height(URDF, load_pose(JOINTS, pose))["pelvis_height"]


def test_home_standing_pelvis_height():
    assert _height("home_standing") == pytest.approx(1.0006, abs=0.002)


def test_home2_is_lower_than_home_standing():
    assert _height("home2") < _height("home_standing") - 0.01


def test_default_paths_exist():
    urdf, joints = default_paths()
    assert os.path.isfile(urdf)
    assert os.path.isfile(joints)
