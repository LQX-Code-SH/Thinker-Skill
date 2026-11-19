"""robosim 后端适配器：robot_sim ROS 话题 → BodyAPI / SensorAPI。

与真机共用 TienkungDexBody 的关节表、限位、IK 与手部映射，只替换
"命令怎么发、状态怎么收、相机怎么取"这一层。
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import yaml

from ..body import TienkungDexBody

from .facade import SimHead, SimRobot
from .sensor import SimSensor


def _package_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("tienkung_dex_robot"))
    except Exception:
        return Path(__file__).resolve().parents[2]


def load_mapping(path: Optional[str] = None) -> dict:
    """读取 resources/sim_mapping.yaml（话题、增益、头部符号、手预设）。"""
    target = Path(path) if path else _package_dir() / "resources" / "sim_mapping.yaml"
    with open(target, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


class SimAdapter:
    def __init__(self, body, sensor, robot):
        self.body = body
        self.sensor = sensor
        self.robot = robot

    def start(self) -> None:
        pass

    def shutdown(self) -> None:
        for gripper in getattr(self.body, "grippers", {}).values():
            hand = getattr(gripper, "_inspire_hand", None)
            if hand is not None:
                try:
                    hand.stop()
                except Exception:  # noqa: BLE001 - 关停路径不抛错
                    pass


def create_adapter(node, *, backend: str = "robosim",
                   hand_variant: str = "inspire", hand_side: str = "right",
                   hand_dof: Optional[int] = None, enable: Any = None,
                   joints_table_path: Any = None,
                   camera: str = "/ob_camera_head",
                   urdf_path: Any = None, camera_k: Any = None,
                   pelvis_height: float = 0.0,
                   ik_solver: str = "daqp") -> SimAdapter:
    if backend not in ("robosim", "sim"):
        raise ValueError(f"robosim 适配器只处理 robosim/sim，实际 {backend!r}")

    mapping = load_mapping()
    robot = SimRobot(node, mapping)
    if hand_dof is None:
        hand_dof = 13 if hand_variant == "inspire" else 6
    vendor = "inspire" if hand_variant == "inspire" else "brainco"
    # 因时手与真机同路径：走项目侧 InspireHand（13 通道 angle_set/angle_actual）
    inspire_hands = None
    if vendor == "inspire":
        if node is None:
            raise ValueError("inspire 手需要传入 ROS node 以创建话题")
        from ..inspire_hand import InspireHand
        inspire_hands = {}
        for side in TienkungDexBody.SIDES:
            hand = InspireHand(node, side)
            hand.start()
            inspire_hands[side] = hand

    body = TienkungDexBody(
        robot,
        hand_side=hand_side,
        hand_dof=int(hand_dof),
        hand_vendor=vendor,
        hand_variant=hand_variant,
        inspire_hands=inspire_hands,
        urdf_path=urdf_path,
        pelvis_height=pelvis_height,
        ik_solver=ik_solver,
    )
    body.head = SimHead(robot, mapping.get("head") or {})

    prefix = str(camera or "/ob_camera_head").rstrip("/")
    name = prefix.lstrip("/") or "ob_camera_head"
    # 仿真相机内参以 camera_info 为准（640x480 与真机不同）；
    # 传 None 让 SimSensor 用话题里的 K，而不是 configs/robot.yaml 的真机值
    sensor = SimSensor(node, robot, camera=name, prefix=prefix,
                       urdf_path=urdf_path, k=None,
                       pelvis_height=pelvis_height)
    return SimAdapter(body, sensor, robot)
