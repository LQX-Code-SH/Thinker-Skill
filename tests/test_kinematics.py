"""tienkung_dex 头部相机 FK 测试（真实 URDF，无 ROS）。"""
import os

import numpy as np
import pytest

from tienkung_dex_robot.kinematics import HeadCameraChain

URDF = (
    "/home/liu/work/UBTECH-IL-LAB/ubt_sim/assets/robots/tienkung_dex/"
    "urdf/tienkung_dex_brainco.urdf"
)

pytestmark = pytest.mark.skipif(not os.path.isfile(URDF), reason="URDF 不可用")


def _chain():
    return HeadCameraChain(URDF)


def test_zero_pose_camera_height():
    T = _chain().cam_to_base({})
    assert T[2, 3] == pytest.approx(0.6, abs=0.1)
    assert abs(T[0, 3]) < 0.15


def test_pitch_moves_camera():
    c = _chain()
    a = c.cam_to_base({})[:3, 3]
    b = c.cam_to_base({"head_pitch": 0.8727})[:3, 3]
    assert not np.allclose(a, b)


def test_point_to_base():
    p = _chain().point_to_base([0.0, 0.0, 1.0], {})
    assert p.shape == (3,)
    assert p[0] > 0.9          # 相机光轴朝前 → base x


def test_pelvis_height_offset():
    c = _chain()
    p0 = c.point_to_base([0.0, 0.0, 1.0], {})
    p1 = c.point_to_base([0.0, 0.0, 1.0], {}, (0.0, 0.0, 0.9))
    assert p1[2] == pytest.approx(p0[2] + 0.9)


class _Waist:
    def get_pose(self):
        return (0.0, 0.0, 0.0)


class _Head:
    def get_pose(self):
        return (0.0, 0.0)


class _Robot:
    waist = _Waist()
    head = _Head()


def test_sensor_transform_to_base():
    from tienkung_dex_robot.sensor import TienkungDexSensor

    sensor = TienkungDexSensor(_Robot(), urdf_path=URDF)
    p = sensor.transform_to_base([0.0, 0.0, 1.0])
    assert p[0] > 0.9
