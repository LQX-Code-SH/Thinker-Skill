"""腿部正运动学：由站姿关节角核算 pelvis 离地高度（标定用）。

链路：pelvis → hip_pitch → hip_roll → hip_yaw → knee → ankle_pitch → ankle_roll。
足底按 URDF 的碰撞圆柱解析最低点（考虑踝 pitch/roll 后的足面姿态），
取双脚最低点，``pelvis_height = -min(sole_z)``。
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from typing import Dict, Mapping

import numpy as np

from .kinematics import _axis_matrix, _origin_matrix, _rpy_matrix

LEG_JOINTS: Dict[str, int] = {
    "hip_pitch_l_joint": 51,
    "hip_roll_l_joint": 52,
    "hip_yaw_l_joint": 53,
    "knee_pitch_l_joint": 54,
    "ankle_pitch_l_joint": 55,
    "ankle_roll_l_joint": 56,
    "hip_pitch_r_joint": 61,
    "hip_roll_r_joint": 62,
    "hip_yaw_r_joint": 63,
    "knee_pitch_r_joint": 64,
    "ankle_pitch_r_joint": 65,
    "ankle_roll_r_joint": 66,
}

FOOT_LINKS = {"left": "ankle_roll_l_link", "right": "ankle_roll_r_link"}


def default_paths():
    """返回 ``(urdf_path, joints_yaml_path)``；优先 ament share，回退源码树。"""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("tienkung_dex_robot")
        base = os.path.join(share, "resources")
    except Exception:
        base = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "resources")
    return (os.path.join(base, "urdf", "tienkung_dex_brainco.urdf"),
            os.path.join(base, "joints.yaml"))


def load_pose(joints_yaml: str, name: str = "home_standing") -> Dict[int, float]:
    """从 joints.yaml 读取姿态快照（键 = 关节 ID，值 = rad）。"""
    import yaml

    with open(joints_yaml, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    pose = data.get(name) or {}
    return {int(k): float(v) for k, v in pose.items()}


def _parent_map(root):
    return {j.find("child").get("link"): j for j in root.findall("joint")}


def _chain(link: str, parent_of: Mapping) -> list:
    out = []
    while link in parent_of:
        joint = parent_of[link]
        out.append(joint)
        link = joint.find("parent").get("link")
    return list(reversed(out))


def forward_kinematics(root, link: str, angles: Mapping[int, float]) -> np.ndarray:
    """pelvis → ``link`` 的 4×4 变换。"""
    parent_of = _parent_map(root)
    T = np.eye(4)
    for joint in _chain(link, parent_of):
        T = T @ _origin_matrix(joint)
        if joint.get("type") == "revolute":
            joint_id = LEG_JOINTS.get(joint.get("name"), -1)
            T = T @ _axis_matrix(joint, float(angles.get(joint_id, 0.0)))
    return T


def _foot_cylinders(root, link_name: str):
    link = next((l for l in root.findall("link") if l.get("name") == link_name), None)
    if link is None:
        return []
    out = []
    for col in link.findall("collision"):
        cyl = col.find("geometry/cylinder")
        if cyl is None:
            continue
        origin = col.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0")
                                  if origin is not None else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0")
                                  if origin is not None else "0 0 0").split()]
        out.append((np.array(xyz), rpy,
                    float(cyl.get("radius")), float(cyl.get("length"))))
    return out


def _cylinder_min_z(T: np.ndarray, xyz, rpy, radius: float, length: float) -> float:
    """圆柱表面在 pelvis 系的最低 z（轴为局部 z，经 collision origin 旋转）。"""
    R, t = T[:3, :3], T[:3, 3]
    center = R @ np.asarray(xyz, dtype=float) + t
    axis = R @ _rpy_matrix(*rpy) @ np.array([0.0, 0.0, 1.0])
    return float(center[2] - abs(axis[2]) * length / 2.0
                 - radius * np.sqrt(max(0.0, 1.0 - axis[2] ** 2)))


def compute_pelvis_height(urdf_path: str, angles: Mapping[int, float]) -> dict:
    """返回 ``{pelvis_height, feet:{side:{ankle_z, sole_z, link}}}``（米）。"""
    root = ET.parse(urdf_path).getroot()
    feet = {}
    for side, link in FOOT_LINKS.items():
        T = forward_kinematics(root, link, angles)
        cylinders = _foot_cylinders(root, link)
        if not cylinders:
            raise ValueError(f"{link} 缺少足底碰撞圆柱")
        feet[side] = {
            "link": link,
            "ankle_z": float(T[2, 3]),
            "sole_z": min(_cylinder_min_z(T, *c) for c in cylinders),
        }
    lowest = min(f["sole_z"] for f in feet.values())
    return {"pelvis_height": -lowest, "feet": feet}
