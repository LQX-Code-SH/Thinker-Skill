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


def test_reachability_sphere():
    ik = _ik()
    shoulder = ik.shoulder_position()
    assert ik.is_reachable(shoulder + np.array([0.0, 0.0, -0.2])) is True
    assert ik.is_reachable(shoulder + np.array([5.0, 0.0, 0.0])) is False


def test_tcp_offset_softens_reach_bound():
    ik = ArmIK(URDF, side="left", base_offset=(0, 0, 0.92),
               tcp_offset=(0.0, 0.0, -0.10), reach_margin=0.03)
    shoulder = ik.shoulder_position()
    edge = ik.max_reach - ik.reach_margin
    target = shoulder + np.array([edge + 0.05, 0.0, 0.0])
    assert ik.is_reachable(target, tcp=False) is False
    assert ik.is_reachable(target, tcp=True) is None


def test_daqp_solves_near_limit():
    """限位附近的代表性目标：DAQP 收敛（max_iter=100）。"""
    target = np.array([0.30, 0.20, 0.90])
    ik = _ik("daqp")
    q, ok = ik.solve(target)
    assert ok
    assert np.linalg.norm(ik.fk(q) - target) < 2e-3


def test_dls_explicit_solver():
    q, _ = _ik().solve([0.30, 0.20, 0.90], solver="dls")
    assert q.shape == (7,)
