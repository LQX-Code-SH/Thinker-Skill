"""robosim RGB-D 传感器：复用真机 SensorAPI，位姿优先用桥发布的 /pose。"""
from __future__ import annotations

from dataclasses import replace
from typing import Any

from ..sensor import TienkungDexSensor

from . import plant


def _stamp_sec(stamp: Any) -> float:
    if stamp is None:
        return 0.0
    return float(getattr(stamp, "sec", 0)) + \
        float(getattr(stamp, "nanosec", 0)) / 1e9


class SimSensor(TienkungDexSensor):
    """真机传感器的子类：相机内参与标定来自仿真话题。

    - camera_info 的 K 覆盖 configs 里的真机内参（仿真 640x480 与真机不同）
    - transform_to_base 优先用 /pose（相机光学系在 world 下，fixed_base 时≈base），
      没有 /pose 时回退到真机的 URDF 链
    """

    def __init__(self, node, robot: Any, *, camera: str = "ob_camera_head",
                 prefix: str = "/ob_camera_head", **kwargs):
        if kwargs.get("k") is None:
            kwargs["k"] = None
        super().__init__(robot, camera=camera, **kwargs)
        from geometry_msgs.msg import PoseStamped

        self._pose = None
        self._pose_sub = node.create_subscription(
            PoseStamped, f"{str(prefix).rstrip('/')}/pose", self._on_pose, 2)

    def _on_pose(self, msg) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self._pose = ((float(p.x), float(p.y), float(p.z)),
                      (float(q.w), float(q.x), float(q.y), float(q.z)),
                      _stamp_sec(msg.header.stamp))

    def rgbd(self, *, timeout: float = 2.0):
        frame = super().rgbd(timeout=timeout)
        if self._k is None:
            stream = self._stream
            k = getattr(stream, "k", None)
            if k:
                self._k = list(k)
                frame = replace(frame, k=list(k))
        return frame

    def transform_to_base(self, point):
        pose = self._pose
        if pose is None:
            return super().transform_to_base(point)
        position, quat, _ = pose
        out = plant.transform_point(point, position, quat)
        # /pose 是 pelvis 系；真机链是 pelvis → base(地面)，补同样的高度偏移
        out[2] += float(self._pelvis_height)
        return out
