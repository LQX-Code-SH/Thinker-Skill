"""臂 IK 测试（真实 URDF，无 ROS）。"""
import os

import numpy as np
import pytest

from tienkung_dex_robot.ik import ArmIK

URDF = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "robots", "tienkung_dex", "resources", "urdf", "tienkung_dex_brainco.urdf")


def _ik(solver="daqp"):
    return ArmIK(URDF, side="left", solver=solver, base_offset=(0, 0, 0.92))


def test_joint_count():
    assert len(_ik().joint_names) == 7


def test_fk_zero_pose():
    p = _ik().fk(np.zeros(7))
    assert p[1] > 0.15          # 左臂 y 偏置
    assert 0.5 < p[2] < 1.0     # 腕在肩下方


@pytest.mark.parametrize("target", [[0.30, 0.20, 0.90], [0.28, 0.22, 0.88]])
def test_ik_reachable(target):
    ik = _ik()
    q, ok = ik.solve(target)
    assert ok
    assert np.linalg.norm(ik.fk(q) - np.array(target)) < 2e-3


def test_ik_unreachable():
    _, ok = _ik().solve([0.40, 0.15, 0.85])   # 超臂展
    assert not ok


def test_daqp_beats_dls_near_limit():
    """限位附近：DAQP 收敛，DLS 明显更差（选型实证）。"""
    target = np.array([0.30, 0.20, 0.90])
    qd, okd = _ik("daqp").solve(target)
    ql, _ = _ik("dls").solve(target)

    assert okd
    assert np.linalg.norm(_ik("daqp").fk(qd) - target) < 2e-3
    assert np.linalg.norm(_ik("dls").fk(ql) - target) > 5e-3


def test_dls_explicit_solver():
    q, _ = _ik().solve([0.30, 0.20, 0.90], solver="dls")
    assert q.shape == (7,)
