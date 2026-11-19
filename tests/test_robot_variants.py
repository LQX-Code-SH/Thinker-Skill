"""tienkung_dex 手部变体与机型资源测试。"""
import os

import yaml

from thinker_skill_core.app import load_capabilities

ROBOT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "robots", "tienkung_dex")


def test_brainco_has_gripper():
    caps = load_capabilities(ROBOT_DIR, "brainco")
    assert {"body.arm", "body.gripper"} <= caps


def test_inspire_has_gripper():
    caps = load_capabilities(ROBOT_DIR, "inspire")
    assert "body.gripper" in caps


def test_fixed_has_no_gripper():
    caps = load_capabilities(ROBOT_DIR, "fixed")
    assert "body.gripper" not in caps
    assert "body.arm" in caps


def test_robosim_capabilities_remove_base():
    caps = load_capabilities(ROBOT_DIR, "inspire", "robosim")
    assert "body.base" not in caps
    assert "sensor.pointcloud" not in caps
    assert "sensor.imu" not in caps
    assert {"body.arm", "body.head", "body.waist",
            "body.gripper", "sensor.rgbd"} <= caps


def test_robosim_uses_ros_adapter(monkeypatch):
    from tienkung_dex_robot import adapter
    from tienkung_dex_robot.robosim import adapter as sim_adapter

    seen = {}

    def _fake_create(node, **kwargs):
        seen.update(kwargs)
        return object()

    monkeypatch.setattr(sim_adapter, "create_adapter", _fake_create)
    adapter.create_adapter(None, backend="robosim", hand_variant="inspire")
    assert seen["hand_side"] == "right"
    assert seen["hand_dof"] == 13


def test_arm_capability_per_side():
    """双臂按侧声明（D-021）：左右臂都在，单侧键也能用于 requires。"""
    for variant in ("brainco", "inspire", "fixed"):
        caps = load_capabilities(ROBOT_DIR, variant)
        assert {"body.arm.left", "body.arm.right"} <= caps, variant


def test_gripper_capability_per_side():
    """双手按侧声明；fixed 假手没有 gripper。"""
    caps = load_capabilities(ROBOT_DIR, "brainco")
    assert {"body.gripper.left", "body.gripper.right"} <= caps

    fixed = load_capabilities(ROBOT_DIR, "fixed")
    assert "body.gripper.left" not in fixed
    assert "body.gripper.right" not in fixed


def test_urdf_default_in_package():
    from tienkung_dex_robot.kinematics import default_urdf_path
    assert os.path.isfile(default_urdf_path())


def test_joints_yaml_matches_constants():
    from tienkung_dex_robot import joints as J

    with open(os.path.join(ROBOT_DIR, "resources", "joints.yaml"),
              encoding="utf-8") as fh:
        data = yaml.safe_load(fh)

    for group, const in (("arm", J.ARM_JOINTS),
                         ("head", J.HEAD_JOINTS),
                         ("waist", J.WAIST_JOINTS)):
        for item in data[group]:
            assert const[item["name"]] == item["id"], f"{group}/{item['name']}"
