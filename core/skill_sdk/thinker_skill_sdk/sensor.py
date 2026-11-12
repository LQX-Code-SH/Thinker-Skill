"""感知能力抽象。

``core/sensor`` 只做数据通道；检测/估计/识别算法随技能包走（D-007）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol


@dataclass(frozen=True)
class Frame:
    """一帧 RGB-D。

    - ``color``: HxWx3 BGR ``uint8``（实现侧保证）
    - ``depth``: HxW ``uint16``，单位**毫米**
    - ``stamp``: 秒
    - ``frame_id``: 坐标/相机命名空间
    """

    color: Any
    depth: Any
    stamp: float
    frame_id: str = ""


class SensorAPI(Protocol):
    """感知数据通道。"""

    def rgbd(self, *, timeout: float = 2.0) -> Frame: ...
    def transform_to_base(self, pose: Any) -> Any: ...
