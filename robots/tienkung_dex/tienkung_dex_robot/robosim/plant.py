"""robosim 后端的纯逻辑：关节映射、限位、四元数、图像转换。

不依赖 ROS / MuJoCo，宿主机（Humble）也能直接单测。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

# robot_sim manip_core.joint_command 的电机 id ↔ 关节名（与桥保持同步）
JOINT_NAME_BY_ID: dict[int, str] = {
    1: "head_yaw_joint",
    2: "head_pitch_joint",
    11: "shoulder_pitch_l_joint",
    12: "shoulder_roll_l_joint",
    13: "shoulder_yaw_l_joint",
    14: "elbow_pitch_l_joint",
    15: "wrist_pitch_l_joint",
    16: "wrist_roll_l_joint",
    17: "wrist_yaw_l_joint",
    21: "shoulder_pitch_r_joint",
    22: "shoulder_roll_r_joint",
    23: "shoulder_yaw_r_joint",
    24: "elbow_pitch_r_joint",
    25: "wrist_pitch_r_joint",
    26: "wrist_roll_r_joint",
    27: "wrist_yaw_r_joint",
    31: "waist_pitch_joint",
    32: "waist_roll_joint",
    33: "waist_yaw_joint",
}

ARM_IDS = tuple(range(11, 18)) + tuple(range(21, 28))
WAIST_IDS = (31, 32, 33)
HEAD_IDS = (1, 2)


@dataclass
class JointState:
    pos: float = 0.0
    vel: float = 0.0
    effort: float = 0.0


def clamp(value: float, limits: Sequence[float]) -> float:
    lo, hi = float(limits[0]), float(limits[1])
    return max(lo, min(hi, float(value)))


def quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """四元数 (w,x,y,z) → 3×3 旋转矩阵。"""
    n = w * w + x * x + y * y + z * z
    if n <= 0.0:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def transform_point(point: Sequence[float], position: Sequence[float],
                    quat: Sequence[float]) -> np.ndarray:
    """相机光学系 → world/base（fixed_base 时 world≈base）。"""
    p = np.asarray(point, dtype=float).reshape(3)
    t = np.asarray(position, dtype=float).reshape(3)
    w, x, y, z = (float(v) for v in quat)
    return quat_to_matrix(w, x, y, z) @ p + t


def rgb_to_bgr(image) -> np.ndarray:
    """rgb8 HxWx3 → BGR；其他形状原样返回。"""
    arr = np.asarray(image)
    if arr.ndim == 3 and arr.shape[2] == 3:
        return np.ascontiguousarray(arr[:, :, ::-1])
    return np.ascontiguousarray(arr)


def depth_to_mm(depth, encoding: str = "16UC1") -> np.ndarray:
    """16UC1 已是毫米；32FC1 按米 ×1000 夹到 uint16。"""
    arr = np.asarray(depth)
    if "32F" in str(encoding).upper():
        return np.clip(np.nan_to_num(arr, nan=0.0) * 1000.0, 0, 65535).astype(
            np.uint16)
    return arr.astype(np.uint16, copy=False)
