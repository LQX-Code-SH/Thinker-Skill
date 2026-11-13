"""tienkung_dex 适配器工厂。

把 SDK 门面（``create_robot``）包装成对外的 ``body`` / ``sensor``。
SDK 在函数内延迟导入，未安装时 import 本模块不报错。

手部变体（hand_variant）：
- ``brainco``：强脑 6 DOF 灵巧手
- ``inspire``：因时 13 关节灵巧手
- ``fixed``：拳头假手（不启用 hand 子系统，无 ``body.gripper`` 能力）
"""
from __future__ import annotations

from typing import Any, Dict, Optional, Set

from .body import TienkungDexBody
from .sensor import DEFAULT_CAMERA, TienkungDexSensor

# 与 SDK backends.real.factory 的子系统 key 对齐
ALL_SUBSYSTEMS = {
    "joint", "safety", "walk", "camera", "panorama", "hand",
    "audio", "imu", "lidar", "gps", "power", "light", "sbus", "serial",
}

HAND_VARIANTS: Dict[str, Dict[str, Any]] = {
    "brainco": {"hand_vendor": "brainco", "enable_hand": True, "hand_dof": 6},
    "inspire": {"hand_vendor": "inspire", "enable_hand": True, "hand_dof": 13},
    "fixed": {"hand_vendor": "brainco", "enable_hand": False, "hand_dof": 0},
}


def _import_create_robot():
    try:
        from tienkung_dex import create_robot  # noqa: F401
    except Exception as exc:  # pragma: no cover - 依赖外部 SDK
        raise RuntimeError(
            "tienkung_dex SDK 不可用：请安装 xhumanoid_sdk 并 source ROS 环境"
        ) from exc
    return create_robot


def _enable_for(variant: Dict[str, Any],
                enable: Optional[Set[str]]) -> Optional[Set[str]]:
    """按变体决定 SDK 的 enable 集合；None 表示全启用。"""
    if enable is not None:
        out = set(enable)
        if variant["enable_hand"]:
            out.add("hand")
        else:
            out.discard("hand")
        return out
    if variant["enable_hand"]:
        return None
    return set(ALL_SUBSYSTEMS) - {"hand"}


class TienkungDexAdapter:
    """同时暴露 body / sensor 的机型适配器。"""

    def __init__(
        self,
        robot: Any,
        *,
        hand_side: str = "right",
        hand_dof: int = 6,
        camera: str = DEFAULT_CAMERA,
        urdf_path: Optional[str] = None,
        camera_k: Optional[list] = None,
        pelvis_height: float = 0.0,
        hand_variant: str = "brainco",
        ik_solver: str = "daqp",
        ik_max_iter: int = 100,
        ik_tol: float = 1e-3,
        ik_damping: float = 0.08,
        tcp_offset=None,
        reach_margin: float = 0.03,
    ):
        self.raw = robot
        self.hand_variant = hand_variant
        self.body = TienkungDexBody(
            robot, hand_side=hand_side, hand_dof=hand_dof,
            urdf_path=urdf_path, pelvis_height=pelvis_height,
            ik_solver=ik_solver, ik_max_iter=ik_max_iter,
            ik_tol=ik_tol, ik_damping=ik_damping,
            tcp_offset=tcp_offset, reach_margin=reach_margin)
        self.sensor = TienkungDexSensor(
            robot, camera=camera, urdf_path=urdf_path, k=camera_k,
            pelvis_height=pelvis_height)

    def start(self) -> None:
        self.raw.start()

    def shutdown(self) -> None:
        self.raw.shutdown()

    def emergency_stop(self) -> None:
        """软急停：停底盘速度流；关节保持最后位置（位置模式天然保持）。"""
        try:
            self.body.base.stop()
        except Exception:  # noqa: BLE001 - 急停路径不能再抛错
            pass


def create_adapter(
    node,
    *,
    backend: str = "real",
    hand_variant: str = "brainco",
    hand_side: str = "right",
    hand_dof: Optional[int] = None,
    enable: Optional[Set[str]] = None,
    joints_table_path: Optional[str] = None,
    camera: str = DEFAULT_CAMERA,
    urdf_path: Optional[str] = None,
    camera_k: Optional[list] = None,
    pelvis_height: float = 0.0,
    ik_solver: str = "daqp",
    ik_max_iter: int = 100,
    ik_tol: float = 1e-3,
    ik_damping: float = 0.08,
    tcp_offset=None,
    reach_margin: float = 0.03,
) -> TienkungDexAdapter:
    """创建适配器（未 start；调用方需 ``adapter.start()``）。"""
    variant = HAND_VARIANTS.get(hand_variant)
    if variant is None:
        raise ValueError(
            f"未知 hand_variant: {hand_variant!r}（可选 brainco/inspire/fixed）")

    dof = variant["hand_dof"] if hand_dof is None else hand_dof
    create_robot = _import_create_robot()
    robot = create_robot(
        node,
        backend=backend,
        hand_vendor=variant["hand_vendor"],
        enable=_enable_for(variant, enable),
        joints_table_path=joints_table_path,
    )
    return TienkungDexAdapter(
        robot, hand_side=hand_side, hand_dof=dof,
        camera=camera, urdf_path=urdf_path, camera_k=camera_k,
        pelvis_height=pelvis_height, hand_variant=hand_variant,
        ik_solver=ik_solver, ik_max_iter=ik_max_iter,
        ik_tol=ik_tol, ik_damping=ik_damping,
        tcp_offset=tcp_offset, reach_margin=reach_margin,
    )
