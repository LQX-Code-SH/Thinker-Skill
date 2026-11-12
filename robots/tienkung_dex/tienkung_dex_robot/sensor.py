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
                 poll: float = 0.02, urdf_path: Any = None, k: Any = None,
                 pelvis_height: float = 0.0):
        self._robot = robot
        self._camera = camera
        self._poll = poll
        self._urdf_path = urdf_path
        self._k = k
        self._pelvis_height = float(pelvis_height)
        self._chain = None

    def _chain_obj(self):
        if self._chain is None:
            from .kinematics import HeadCameraChain, default_urdf_path
            path = (self._urdf_path
                    or os.environ.get("THINKER_URDF")
                    or default_urdf_path())
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
        from .joints import HEAD_JOINTS, WAIST_JOINTS

        chain = self._chain_obj()
        # SDK 的 RealJointGroup 只提供 get_states()，没有 get_pose()
        waist_states = self._robot.waist.get_states()
        head_states = self._robot.head.get_states()

        def _pos(states, joint_id):
            return float(states[joint_id].pos) if joint_id in states else 0.0

        angles = {
            "waist_pitch": _pos(waist_states, WAIST_JOINTS["waist_pitch"]),
            "waist_roll": _pos(waist_states, WAIST_JOINTS["waist_roll"]),
            "waist_yaw": _pos(waist_states, WAIST_JOINTS["waist_yaw"]),
            "head_yaw": _pos(head_states, HEAD_JOINTS["head_yaw"]),
            "head_pitch": _pos(head_states, HEAD_JOINTS["head_pitch"]),
        }
        return chain.point_to_base(
            point, angles, (0.0, 0.0, self._pelvis_height))
