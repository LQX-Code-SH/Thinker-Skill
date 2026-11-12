"""天工 DEX 关节表（与 SDK ``python/config/joints.yaml`` 一致）。

- 关节名（snake_case）→ SDK 关节 ID
- limits 单位 rad；库本身不做限幅，适配层必须按此夹紧
"""
from __future__ import annotations

from typing import Dict, Tuple

ARM_JOINTS: Dict[str, int] = {
    "left_shoulder_pitch": 11,
    "left_shoulder_roll": 12,
    "left_shoulder_yaw": 13,
    "left_elbow_pitch": 14,
    "left_wrist_yaw": 15,
    "left_wrist_pitch": 16,
    "left_wrist_roll": 17,
    "right_shoulder_pitch": 21,
    "right_shoulder_roll": 22,
    "right_shoulder_yaw": 23,
    "right_elbow_pitch": 24,
    "right_wrist_yaw": 25,
    "right_wrist_pitch": 26,
    "right_wrist_roll": 27,
}

HEAD_JOINTS: Dict[str, int] = {"head_yaw": 1, "head_pitch": 2}

WAIST_JOINTS: Dict[str, int] = {
    "waist_pitch": 31,
    "waist_roll": 32,
    "waist_yaw": 33,
}

ARM_LIMITS: Dict[int, Tuple[float, float]] = {
    11: (-2.8449, 2.8449),
    12: (-0.1920, 3.3336),
    13: (-2.8449, 2.8449),
    14: (-2.5482, 0.1920),
    15: (-2.8449, 2.8449),
    16: (-1.3265, 1.3265),
    17: (-1.3265, 1.3265),
    21: (-2.8449, 2.8449),
    22: (-3.3336, 0.1920),
    23: (-2.8449, 2.8449),
    24: (-2.5482, 0.1920),
    25: (-2.8449, 2.8449),
    26: (-1.3265, 1.3265),
    27: (-1.3265, 1.3265),
}

HEAD_LIMITS: Dict[int, Tuple[float, float]] = {
    1: (-1.0472, 1.0472),
    2: (-0.2618, 1.0123),
}

WAIST_LIMITS: Dict[int, Tuple[float, float]] = {
    31: (-0.5236, 0.8727),
    32: (-0.4363, 0.4363),
    33: (-2.6180, 3.2289),
}


def clamp(value: float, limits: Tuple[float, float]) -> float:
    lo, hi = limits
    return max(lo, min(hi, float(value)))
