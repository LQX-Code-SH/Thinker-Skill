"""tienkung_dex 适配器工厂。

把 SDK 门面（``create_robot``）包装成对外的 ``body`` / ``sensor``。
SDK 在函数内延迟导入，未安装时 import 本模块不报错。
"""
from __future__ import annotations

from typing import Any, Optional, Set

from .body import TienkungDexBody
from .sensor import DEFAULT_CAMERA, TienkungDexSensor


def _import_create_robot():
    try:
        from tienkung_dex import create_robot  # noqa: F401
    except Exception as exc:  # pragma: no cover - 依赖外部 SDK
        raise RuntimeError(
            "tienkung_dex SDK 不可用：请安装 xhumanoid_sdk 并 source ROS 环境"
        ) from exc
    return create_robot


class TienkungDexAdapter:
    """同时暴露 body / sensor 的机型适配器。"""

    def __init__(
        self,
        robot: Any,
        *,
        hand_side: str = "right",
        hand_dof: int = 6,
        camera: str = DEFAULT_CAMERA,
    ):
        self.raw = robot
        self.body = TienkungDexBody(robot, hand_side=hand_side, hand_dof=hand_dof)
        self.sensor = TienkungDexSensor(robot, camera=camera)

    def start(self) -> None:
        self.raw.start()

    def shutdown(self) -> None:
        self.raw.shutdown()


def create_adapter(
    node,
    *,
    backend: str = "real",
    hand_vendor: str = "brainco",
    hand_side: str = "right",
    hand_dof: int = 6,
    enable: Optional[Set[str]] = None,
    joints_table_path: Optional[str] = None,
    camera: str = DEFAULT_CAMERA,
) -> TienkungDexAdapter:
    """创建适配器（未 start；调用方需 ``adapter.start()``）。"""
    create_robot = _import_create_robot()
    robot = create_robot(
        node,
        backend=backend,
        hand_vendor=hand_vendor,
        enable=enable,
        joints_table_path=joints_table_path,
    )
    return TienkungDexAdapter(
        robot, hand_side=hand_side, hand_dof=hand_dof, camera=camera
    )
