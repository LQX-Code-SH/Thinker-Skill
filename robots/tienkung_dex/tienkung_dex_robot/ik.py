"""臂 IK：URDF 链 + 数值雅可比 + DLS / QP(DAQP)。

链：waist(yaw/roll/pitch) → shoulder(pitch/roll/yaw) → elbow(pitch/yaw)
    → wrist(pitch/roll)。腰角度已知，只解 7 个臂关节。

两种任务：

- **位置**（``solve``）：3 维任务，只管腕部位置，姿态由求解器自由给出；
- **位姿**（``solve_pose``）：6 维任务，位置 + 姿态同时收敛，供抓取用。

算法：``dls`` 无约束阻尼最小二乘（纯 numpy）；``daqp`` 每步解带关节限位
约束的 QP（延迟导入 daqp，缺失时降级 dls）。姿态误差统一取旋转向量
（``rotvec``），与位置误差拼成 6 维残差。
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


# ---------- 姿态工具（不引入 scipy） ----------
def quat_to_matrix(quaternion: Sequence[float]) -> np.ndarray:
    """四元数 ``(x, y, z, w)`` → 3×3 旋转矩阵（内部归一化）。"""
    x, y, z, w = (float(v) for v in quaternion)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """3×3 旋转矩阵 → 四元数 ``(x, y, z, w)``（Shepperd 法，``w >= 0``）。"""
    m = np.asarray(R, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    if q[3] < 0.0:
        q = -q
    return q / np.linalg.norm(q)


def matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    """3×3 旋转矩阵 → 旋转向量（轴 × 角，弧度；``|角| <= pi``）。"""
    q = matrix_to_quat(R)
    vec = q[:3]
    norm = float(np.linalg.norm(vec))
    if norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(norm, q[3])
    return vec / norm * angle


class ArmIK:
    """位置/位姿 IK：base 系目标 → 7 个臂关节角。"""

    def __init__(self, urdf_path: str, *, side: str = "left",
                 tol: float = 1e-3, rot_tol: float = 0.02, max_iter: int = 100,
                 damping: float = 0.08, solver: str = "daqp",
                 base_offset=(0.0, 0.0, 0.0),
                 tcp_offset=(0.0, 0.0, 0.0),
                 reach_margin: float = 0.03):
        root = ET.parse(urdf_path).getroot()
        joints = {j.get("name"): j for j in root.findall("joint")}
        self.joint_names = list(ARM_JOINTS[side])
        self._arm = [joints[n] for n in self.joint_names]
        self._waist = [(joints[n], key) for n, key in WAIST_JOINTS]
        self._tol = tol
        self._rot_tol = rot_tol
        self._max_iter = max_iter
        self._damping = damping
        self.solver = solver
        self.base_offset = np.asarray(base_offset, dtype=float)
        self.tcp_offset = np.asarray(tcp_offset, dtype=float)
        self.reach_margin = float(reach_margin)
        suffix = "l" if side == "left" else "r"
        self._shoulder_link = f"shoulder_pitch_{suffix}_link"
        self._parent_of = {j.find("child").get("link"): j
                           for j in root.findall("joint")}

        self.lower = np.array([float(j.find("limit").get("lower")) for j in self._arm])
        self.upper = np.array([float(j.find("limit").get("upper")) for j in self._arm])

        # 肩 → 腕的最大展开（各段 origin 长度之和，上界）
        total = 0.0
        for joint in self._arm[1:]:
            origin = joint.find("origin")
            if origin is not None:
                total += float(np.linalg.norm(
                    [float(v) for v in origin.get("xyz", "0 0 0").split()]))
        self.max_reach = total

    # ---------- 正运动学 ----------
    def fk_pose(self, q: Sequence[float], waist=(0.0, 0.0, 0.0)
                ) -> Tuple[np.ndarray, np.ndarray]:
        """腕 roll 末端在 base 系的 ``(位置, 3×3 旋转)``。

        ``waist`` = ``(pitch, roll, yaw)``；返回的旋转即腕 roll 帧姿态，
        工具（TCP）偏移在该帧下定义。
        """
        wp, wr, wy = waist
        angles = {"waist_pitch": wp, "waist_roll": wr, "waist_yaw": wy}
        T = np.eye(4)
        for joint, key in self._waist:
            T = T @ _origin_matrix(joint) @ _axis_matrix(joint, angles[key])
        for joint, qi in zip(self._arm, q):
            T = T @ _origin_matrix(joint) @ _axis_matrix(joint, qi)
        return T[:3, 3] + self.base_offset, T[:3, :3].copy()

    def fk(self, q: Sequence[float], waist=(0.0, 0.0, 0.0)) -> np.ndarray:
        """腕 roll 末端在 base 系的位置。``waist`` = (pitch, roll, yaw)。"""
        return self.fk_pose(q, waist)[0]

    # ---------- 可达性预检 ----------
    def _chain_to(self, link: str) -> list:
        out = []
        while link in self._parent_of:
            joint = self._parent_of[link]
            out.append(joint)
            link = joint.find("parent").get("link")
        return list(reversed(out))

    def shoulder_position(self, waist=(0.0, 0.0, 0.0)) -> np.ndarray:
        """当前腰角下肩部（shoulder_pitch 关节原点）在 base 系的位置。"""
        wp, wr, wy = waist
        waist_angles = {"waist_pitch": wp, "waist_roll": wr, "waist_yaw": wy}
        key_of = {
            "waist_yaw_joint": "waist_yaw",
            "waist_roll_joint": "waist_roll",
            "waist_pitch_joint": "waist_pitch",
        }
        T = np.eye(4)
        for joint in self._chain_to(self._shoulder_link):
            T = T @ _origin_matrix(joint)
            if joint.get("type") == "revolute" and joint.get("name") in key_of:
                T = T @ _axis_matrix(
                    joint, waist_angles[key_of[joint.get("name")]])
        return T[:3, 3] + self.base_offset

    def reach_bounds(self, target, *, tcp: bool = False,
                     waist=(0.0, 0.0, 0.0)):
        """目标对应的腕部所需距离区间 ``(d_min, d_max)``。

        ``tcp=True`` 时按工具偏移范数放宽/收紧：工具朝向目标时腕部可近
        ``|offset|``，背向时需远 ``|offset|``。
        """
        d = float(np.linalg.norm(
            np.asarray(target, dtype=float) - self.shoulder_position(waist)))
        offset = float(np.linalg.norm(self.tcp_offset)) if tcp else 0.0
        return max(0.0, d - offset), d + offset

    def is_reachable(self, target, *, tcp: bool = False,
                     waist=(0.0, 0.0, 0.0),
                     margin: Optional[float] = None) -> Optional[bool]:
        """球面预检：``False`` 必不可达 / ``True`` 必在臂展内 / ``None`` 不确定。"""
        d_min, d_max = self.reach_bounds(target, tcp=tcp, waist=waist)
        radius = self.max_reach - (
            self.reach_margin if margin is None else float(margin))
        if d_min > radius:
            return False
        if d_max <= radius:
            return True
        return None

    def tcp_to_wrist(self, target, waist=(0.0, 0.0, 0.0)) -> np.ndarray:
        """把 TCP 目标近似折算为腕部目标（工具沿肩→目标方向）。"""
        p = np.asarray(target, dtype=float)
        vec = p - self.shoulder_position(waist)
        norm = float(np.linalg.norm(vec))
        offset = float(np.linalg.norm(self.tcp_offset))
        if norm < 1e-9 or offset <= 0.0:
            return p
        return p - vec / norm * offset

    def tcp_pose_to_wrist(self, position, rotation) -> Tuple[np.ndarray, np.ndarray]:
        """TCP 位姿折算为腕部位姿：``p_wrist = p_tcp - R_tcp @ tcp_offset``。

        ``tcp_offset`` 定义在腕 roll 帧下（配置 ``end_effector.tcp_offset``），
        因此姿态直接沿用 TCP 姿态。相比 ``tcp_to_wrist`` 的"沿肩→目标"方向
        近似，这里用目标姿态旋转偏移，姿态与位置一致。
        """
        R = np.asarray(rotation, dtype=float)
        p = np.asarray(position, dtype=float)
        return p - R @ self.tcp_offset, R.copy()

    def _jacobian(self, q, waist, eps=1e-4) -> np.ndarray:
        p0 = self.fk(q, waist)
        J = np.zeros((3, len(q)))
        for i in range(len(q)):
            q2 = np.array(q, dtype=float)
            q2[i] += eps
            J[:, i] = (self.fk(q2, waist) - p0) / eps
        return J

    def _jacobian6(self, q, waist, eps=1e-4) -> np.ndarray:
        """6×n 雅可比：上 3 行线速度、下 3 行角速度（base 系）。"""
        p0, R0 = self.fk_pose(q, waist)
        J = np.zeros((6, len(q)))
        for i in range(len(q)):
            q2 = np.array(q, dtype=float)
            q2[i] += eps
            p1, R1 = self.fk_pose(q2, waist)
            J[:3, i] = (p1 - p0) / eps
            J[3:, i] = matrix_to_rotvec(R1 @ R0.T) / eps
        return J

    def pose_error(self, q, position, quaternion,
                   waist=(0.0, 0.0, 0.0)) -> Tuple[np.ndarray, np.ndarray]:
        """当前位姿相对目标的 ``(位置误差向量, 姿态误差旋转向量)``。"""
        p, R = self.fk_pose(q, waist)
        e_p = np.asarray(position, dtype=float) - p
        e_r = matrix_to_rotvec(quat_to_matrix(quaternion) @ R.T)
        return e_p, e_r

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

    # ---------- 位姿求解（位置 + 姿态） ----------
    def _pose_residual(self, q, target_p, target_R, waist) -> np.ndarray:
        p, R = self.fk_pose(q, waist)
        return np.concatenate(
            [target_p - p, matrix_to_rotvec(target_R @ R.T)])

    def _pose_converged(self, e: np.ndarray) -> bool:
        return (float(np.linalg.norm(e[:3])) < self._tol
                and float(np.linalg.norm(e[3:])) < self._rot_tol)

    def _solve_dls_pose(self, target_p, target_R, seed, waist):
        q = seed.copy()
        lam2 = self._damping ** 2
        for _ in range(self._max_iter):
            e = self._pose_residual(q, target_p, target_R, waist)
            if self._pose_converged(e):
                return q, True
            J = self._jacobian6(q, waist)
            dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(6), e)
            q = np.clip(q + dq, self.lower, self.upper)
        return q, self._pose_converged(
            self._pose_residual(q, target_p, target_R, waist))

    def _solve_daqp_pose(self, target_p, target_R, seed, waist):
        import daqp

        q = seed.copy()
        lam2 = self._damping ** 2
        n = len(q)
        A = np.eye(n)
        for _ in range(self._max_iter):
            e = self._pose_residual(q, target_p, target_R, waist)
            if self._pose_converged(e):
                return q, True
            J = self._jacobian6(q, waist)
            H = 2.0 * (J.T @ J + lam2 * np.eye(n))
            f = -2.0 * (J.T @ e)
            dq, *_ = daqp.solve(H, f, A, self.upper - q, self.lower - q)
            q = np.clip(q + dq, self.lower, self.upper)
        return q, self._pose_converged(
            self._pose_residual(q, target_p, target_R, waist))

    def solve_pose(self, position, quaternion,
                   seed: Optional[Sequence[float]] = None,
                   waist=(0.0, 0.0, 0.0),
                   solver: Optional[str] = None) -> Tuple[np.ndarray, bool]:
        """6-DoF 位姿求解：``quaternion`` 为 ``(x, y, z, w)``，腕 roll 帧。

        返回 ``(q, converged)``；位置容差 ``tol``、姿态容差 ``rot_tol``
        （弧度）需同时满足。
        """
        mode = solver or self.solver
        q = (np.zeros(len(self.joint_names)) if seed is None
             else np.array(seed, dtype=float))
        q = np.clip(q, self.lower, self.upper)
        target_p = np.asarray(position, dtype=float)
        target_R = quat_to_matrix(quaternion)

        if mode == "daqp":
            try:
                return self._solve_daqp_pose(target_p, target_R, q, waist)
            except ImportError:
                pass  # daqp 未安装 → 降级
        return self._solve_dls_pose(target_p, target_R, q, waist)
