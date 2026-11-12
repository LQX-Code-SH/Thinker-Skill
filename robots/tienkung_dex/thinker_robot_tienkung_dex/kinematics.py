"""头部相机正运动学：``pelvis → camera_head_link``（从 URDF 解析）。

链上只有腰 3 轴 + 头 2 轴 + 相机固定外参；前两者的实际角均可从 SDK 读到：
``robot.waist.get_pose()`` / ``robot.head.get_pose()``。
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

# (joint 名, 角度 key)；fixed 关节 key 为 None
CHAIN = (
    ("waist_yaw_joint", "waist_yaw"),
    ("waist_roll_joint", "waist_roll"),
    ("waist_pitch_joint", "waist_pitch"),
    ("head_yaw_joint", "head_yaw"),
    ("head_pitch_joint", "head_pitch"),
    ("camera_head_joint", None),
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
    origin = joint.find("origin")
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if origin is not None:
        xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def _axis_matrix(joint, q):
    axis = joint.find("axis")
    n = np.array([float(v) for v in axis.get("xyz", "0 0 1").split()])
    norm = np.linalg.norm(n)
    if norm <= 0:
        return np.eye(4)
    n = n / norm
    K = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
    R = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)
    T = np.eye(4)
    T[:3, :3] = R
    return T


class HeadCameraChain:
    """从 URDF 读取固定 origin，按关节角算 ``cam→base`` 4×4。"""

    def __init__(self, urdf_path: str):
        joints = {}
        for j in ET.parse(urdf_path).getroot().findall("joint"):
            joints[j.get("name")] = j
        self._chain = []
        for name, key in CHAIN:
            if name not in joints:
                raise KeyError(f"URDF 缺少关节: {name}")
            self._chain.append((joints[name], key))

    def cam_to_base(self, angles: dict) -> np.ndarray:
        """``angles``: {waist_pitch, waist_roll, waist_yaw, head_yaw, head_pitch} rad。"""
        T = np.eye(4)
        for joint, key in self._chain:
            T = T @ _origin_matrix(joint)
            if key is not None and joint.get("type") == "revolute":
                T = T @ _axis_matrix(joint, float(angles.get(key, 0.0)))
        return T

    def point_to_base(self, point, angles: dict,
                      base_offset=(0.0, 0.0, 0.0)) -> np.ndarray:
        """``base_offset``：pelvis → base(地面) 的平移，通常 ``(0,0,pelvis_height)``。"""
        T = self.cam_to_base(angles)
        p = np.asarray(point, dtype=float).reshape(3)
        p_pelvis = (T @ np.append(p, 1.0))[:3]
        return p_pelvis + np.asarray(base_offset, dtype=float)
