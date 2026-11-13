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
    - ``k``: 相机内参 9 元（fx,0,cx,0,fy,cy,0,0,1），可选
    - ``angles``: 取帧时刻的关节角（腰 3 + 头 2），用于精确 cam→base
    """

    color: Any
    depth: Any
    stamp: float
    frame_id: str = ""
    k: Any = None
    angles: Any = None


class SensorAPI(Protocol):
    """感知数据通道。"""

    def rgbd(self, *, timeout: float = 2.0) -> Frame: ...
    def transform_to_base(self, pose: Any, angles: Any = None) -> Any: ...
