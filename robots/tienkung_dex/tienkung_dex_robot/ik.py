"""臂 IK：URDF 链 + 数值雅可比 + DLS / QP(DAQP)。

链：waist(yaw/roll/pitch) → shoulder(pitch/roll/yaw) → elbow(pitch/yaw)
    → wrist(pitch/roll)。腰角度已知，只解 7 个臂关节。

- ``dls``：无约束阻尼最小二乘（纯 numpy）
- ``daqp``：每步解带关节限位约束的 QP（延迟导入 daqp，缺失时降级 dls）
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Optional, Sequence, Tuple

import numpy as np

ARM_JOINTS = {
    "left": (
        "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint", "elbow_yaw_l_joint",
        "wrist_pitch_l_joint", "wrist_roll_l_joint",
    ),
    "right": (
        "shoulder_pitch_r_joint", "shoulder_roll_r_joint", "shoulder_yaw_r_joint",
        "elbow_pitch_r_joint", "elbow_yaw_r_joint",
        "wrist_pitch_r_joint", "wrist_roll_r_joint",
    ),
}

# 腰关节（已知角度，key 对应 waist.get_pose() 的 pitch/roll/yaw）
WAIST_JOINTS = (
    ("waist_yaw_joint", "waist_yaw"),
    ("waist_roll_joint", "waist_roll"),
    ("waist_pitch_joint", "waist_pitch"),
)


def _rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _origin_matrix(joint):
    o = joint.find("origin")
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if o is not None:
        xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def _axis_matrix(joint, q):
    ax = joint.find("axis")
    n = np.array([float(v) for v in ax.get("xyz", "0 0 1").split()])
    norm = np.linalg.norm(n)
    if norm <= 0:
        return np.eye(4)
    n = n / norm
    K = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)
    return T


class ArmIK:
    """位置 IK：base 系目标点 → 7 个臂关节角。"""

    def __init__(self, urdf_path: str, *, side: str = "left",
                 tol: float = 1e-3, max_iter: int = 30,
                 damping: float = 0.08, solver: str = "daqp",
                 base_offset=(0.0, 0.0, 0.0)):
        root = ET.parse(urdf_path).getroot()
        joints = {j.get("name"): j for j in root.findall("joint")}
        self.joint_names = list(ARM_JOINTS[side])
        self._arm = [joints[n] for n in self.joint_names]
        self._waist = [(joints[n], key) for n, key in WAIST_JOINTS]
        self._tol = tol
        self._max_iter = max_iter
        self._damping = damping
        self.solver = solver
        self.base_offset = np.asarray(base_offset, dtype=float)

        self.lower = np.array([float(j.find("limit").get("lower")) for j in self._arm])
        self.upper = np.array([float(j.find("limit").get("upper")) for j in self._arm])

    # ---------- 正运动学 ----------
    def fk(self, q: Sequence[float], waist=(0.0, 0.0, 0.0)) -> np.ndarray:
        """腕 roll 末端在 base 系的位置。``waist`` = (pitch, roll, yaw)。"""
        wp, wr, wy = waist
        angles = {"waist_pitch": wp, "waist_roll": wr, "waist_yaw": wy}
        T = np.eye(4)
        for joint, key in self._waist:
            T = T @ _origin_matrix(joint) @ _axis_matrix(joint, angles[key])
        for joint, qi in zip(self._arm, q):
            T = T @ _origin_matrix(joint) @ _axis_matrix(joint, qi)
        return T[:3, 3] + self.base_offset

    def _jacobian(self, q, waist, eps=1e-4) -> np.ndarray:
        p0 = self.fk(q, waist)
        J = np.zeros((3, len(q)))
        for i in range(len(q)):
            q2 = np.array(q, dtype=float)
            q2[i] += eps
            J[:, i] = (self.fk(q2, waist) - p0) / eps
        return J

    # ---------- 求解器 ----------
    def _solve_dls(self, target, seed, waist):
        q = seed.copy()
        lam2 = self._damping ** 2
        for _ in range(self._max_iter):
            e = target - self.fk(q, waist)
            if np.linalg.norm(e) < self._tol:
                return q, True
            J = self._jacobian(q, waist)
            dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(3), e)
            q = np.clip(q + dq, self.lower, self.upper)
        return q, bool(np.linalg.norm(target - self.fk(q, waist)) < self._tol)

    def _solve_daqp(self, target, seed, waist):
        import daqp

        q = seed.copy()
        lam2 = self._damping ** 2
        n = len(q)
        A = np.eye(n)
        for _ in range(self._max_iter):
            e = target - self.fk(q, waist)
            if np.linalg.norm(e) < self._tol:
                return q, True
            J = self._jacobian(q, waist)
            H = 2.0 * (J.T @ J + lam2 * np.eye(n))
            f = -2.0 * (J.T @ e)
            dq, *_ = daqp.solve(H, f, A, self.upper - q, self.lower - q)
            q = np.clip(q + dq, self.lower, self.upper)
        return q, bool(np.linalg.norm(target - self.fk(q, waist)) < self._tol)

    def solve(self, target, seed: Optional[Sequence[float]] = None,
              waist=(0.0, 0.0, 0.0),
              solver: Optional[str] = None) -> Tuple[np.ndarray, bool]:
        """返回 ``(q, converged)``。"""
        mode = solver or self.solver
        q = (np.zeros(len(self.joint_names)) if seed is None
             else np.array(seed, dtype=float))
        q = np.clip(q, self.lower, self.upper)
        target = np.asarray(target, dtype=float)

        if mode == "daqp":
            try:
                return self._solve_daqp(target, q, waist)
            except ImportError:
                pass  # daqp 未安装 → 降级
        return self._solve_dls(target, q, waist)
