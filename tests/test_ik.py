"""臂 IK 测试（真实 URDF，无 ROS）。"""
import math
import os

import numpy as np
import pytest

from tienkung_dex_robot.ik import (
    ArmIK,
    matrix_to_quat,
    matrix_to_rotvec,
    quat_to_matrix,
)

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


# ---------- 位姿（6-DoF）IK ----------

def _ry(angle_rad):
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(angle_rad):
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_quat_matrix_roundtrip():
    R = _rz(0.3) @ _ry(-math.pi / 2)
    assert np.allclose(quat_to_matrix(matrix_to_quat(R)), R, atol=1e-12)
    assert np.linalg.norm(matrix_to_rotvec(R)) == pytest.approx(
        np.linalg.norm(matrix_to_rotvec(R)))


def test_rotvec_identity_is_zero():
    assert np.linalg.norm(matrix_to_rotvec(np.eye(3))) < 1e-12


def test_fk_pose_matches_fk():
    ik = _ik()
    q = np.array([0.1, 0.2, -0.1, -0.4, 0.05, -0.3, 0.1])
    p, R = ik.fk_pose(q)
    assert np.allclose(p, ik.fk(q))
    # 姿态是正交旋转矩阵
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(R) == pytest.approx(1.0)


@pytest.mark.parametrize("target", [[0.30, 0.20, 0.90], [0.28, 0.22, 0.88]])
def test_solve_pose_downward_grasp(target):
    """朝下抓取姿态（Ry-90°）在可达点收敛，位置与姿态同时满足容差。"""
    ik = _ik()
    R = _ry(-math.pi / 2)
    q, ok = ik.solve_pose(target, matrix_to_quat(R))
    assert ok
    assert np.all(q >= ik.lower - 1e-9) and np.all(q <= ik.upper + 1e-9)
    p, Rf = ik.fk_pose(q)
    assert np.linalg.norm(np.asarray(target) - p) < 2e-3
    assert np.linalg.norm(matrix_to_rotvec(R @ Rf.T)) < 0.02


def test_solve_pose_with_extra_yaw():
    """叠加额外 yaw（抓取偏转）后仍可解。"""
    ik = _ik()
    R = _rz(math.radians(30.0)) @ _ry(-math.pi / 2)
    q, ok = ik.solve_pose([0.28, 0.24, 0.88], matrix_to_quat(R))
    assert ok
    p, Rf = ik.fk_pose(q)
    assert np.linalg.norm(np.array([0.28, 0.24, 0.88]) - p) < 2e-3
    assert np.linalg.norm(matrix_to_rotvec(R @ Rf.T)) < 0.02


def test_solve_pose_dls_path():
    ik = _ik("daqp")
    R = _ry(-math.pi / 2)
    q, ok = ik.solve_pose([0.30, 0.20, 0.90], matrix_to_quat(R), solver="dls")
    assert ok
    assert q.shape == (7,)


def test_solve_pose_unreachable_orientation():
    """夹爪朝上（Ry+90°）超出腕部姿态能力：不收敛。"""
    ik = _ik()
    _, ok = ik.solve_pose([0.30, 0.20, 0.90],
                          matrix_to_quat(_ry(math.pi / 2)))
    assert not ok


def test_pose_error_reports_residual():
    ik = _ik()
    R = _ry(-math.pi / 2)
    q, ok = ik.solve_pose([0.30, 0.20, 0.90], matrix_to_quat(R))
    assert ok
    e_p, e_r = ik.pose_error(q, [0.30, 0.20, 0.90], matrix_to_quat(R))
    assert np.linalg.norm(e_p) < 2e-3
    assert np.linalg.norm(e_r) < 0.02


def test_tcp_pose_to_wrist_uses_target_rotation():
    """TCP 偏置按目标姿态旋转折算，而不是沿肩→目标方向近似。"""
    ik = ArmIK(URDF, side="left", base_offset=(0, 0, 0.92),
               tcp_offset=(0.0, 0.0, -0.10))
    R = _ry(-math.pi / 2)
    tcp = np.array([0.30, 0.20, 0.90])
    p_wrist, R_wrist = ik.tcp_pose_to_wrist(tcp, R)
    assert np.allclose(R_wrist, R)
    # 反算回 TCP 应还原
    assert np.allclose(p_wrist + R_wrist @ ik.tcp_offset, tcp)
    # 姿态不同 → 腕部位置不同（旧近似只按位置算，无法区分）
    _, p_wrist2 = ik.tcp_pose_to_wrist(tcp, np.eye(3))
    assert not np.allclose(p_wrist, p_wrist2)
