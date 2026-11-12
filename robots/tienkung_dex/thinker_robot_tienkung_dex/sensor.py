"""tienkung_dex SensorAPI 实现（真机可用）。

相机：``robot.cameras[namespace].latest() -> CameraFrame``（color BGR uint8，
depth uint16 毫米）。坐标变换需 TF/标定，SDK 不提供，故 ``transform_to_base`` 未实现。
"""
from __future__ import annotations

import os
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

    def __init__(self, robot: Any, *, camera: str = DEFAULT_CAMERA,
                 poll: float = 0.02, urdf_path: Any = None, k: Any = None):
        self._robot = robot
        self._camera = camera
        self._poll = poll
        self._urdf_path = urdf_path
        self._k = k
        self._chain = None

    def _chain_obj(self):
        if self._chain is None:
            from .kinematics import HeadCameraChain
            path = self._urdf_path or os.environ.get("THINKER_URDF")
            if not path:
                raise NotImplementedError(
                    "cam->base 需要 URDF：设置 THINKER_URDF 或传入 urdf_path")
            self._chain = HeadCameraChain(path)
        return self._chain

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
            k=self._k,
        )

    def transform_to_base(self, point):
        chain = self._chain_obj()
        waist = self._robot.waist.get_pose()
        head = self._robot.head.get_pose()
        angles = {
            "waist_pitch": waist[0],
            "waist_roll": waist[1],
            "waist_yaw": waist[2],
            "head_yaw": head[0],
            "head_pitch": head[1],
        }
        return chain.point_to_base(point, angles)
