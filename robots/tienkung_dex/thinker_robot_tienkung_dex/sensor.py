"""tienkung_dex SensorAPI 实现（真机可用）。

相机：``robot.cameras[namespace].latest() -> CameraFrame``（color BGR uint8，
depth uint16 毫米）。坐标变换需 TF/标定，SDK 不提供，故 ``transform_to_base`` 未实现。
"""
from __future__ import annotations

import time
from typing import Any

from thinker_skill_sdk import Frame, SensorAPI

DEFAULT_CAMERA = "ob_camera_head"
CAMERA_NAMESPACES = (
    "ob_camera_head",
    "ob_camera_waist",
    "ob_camera_wrist_left",
    "ob_camera_wrist_right",
)


def _stamp_to_sec(stamp: Any) -> float:
    if stamp is None:
        return 0.0
    if isinstance(stamp, (int, float)):
        return float(stamp)
    sec = getattr(stamp, "sec", None)
    if sec is not None:
        return float(sec) + float(getattr(stamp, "nanosec", 0)) / 1e9
    return 0.0


class TienkungDexSensor(SensorAPI):
    """RGB-D 数据通道。"""

    def __init__(self, robot: Any, *, camera: str = DEFAULT_CAMERA, poll: float = 0.02):
        self._robot = robot
        self._camera = camera
        self._poll = poll

    @property
    def _stream(self):
        cameras = getattr(self._robot, "cameras", {}) or {}
        stream = cameras.get(self._camera)
        if stream is None:
            stream = next(iter(cameras.values()), None)
        return stream

    def rgbd(self, *, timeout: float = 2.0) -> Frame:
        stream = self._stream
        if stream is None:
            raise RuntimeError(f"tienkung_dex: 相机 {self._camera} 不存在")
        deadline = time.monotonic() + timeout
        frame = stream.latest()
        while frame is None and time.monotonic() < deadline:
            time.sleep(self._poll)
            frame = stream.latest()
        if frame is None:
            raise TimeoutError(f"tienkung_dex: 相机 {self._camera} 在 {timeout}s 内无帧")
        return Frame(
            color=frame.color,
            depth=frame.depth,
            stamp=_stamp_to_sec(getattr(frame, "stamp", None)),
            frame_id=getattr(frame, "frame_id", "") or "",
        )

    def transform_to_base(self, pose):
        raise NotImplementedError("坐标变换需 TF/标定，由 core/sensor 提供；SDK 无 TF")
