"""detect_object：低头 → 取帧 → 检测 → 3D 定位 → base 系位置。"""
from __future__ import annotations

import math

from thinker_skill_sdk import ErrorCode, Step, StepResult

from .detector import GroundingDinoDetector
from .geometry import estimate_from_box, estimate_from_mask
from .spatial import in_valid_range, normalize_prompt


class DetectObject:
    name = "detect_object"
    requires = ["body.head", "sensor.rgbd"]
    postcondition = {}
    timeout = 30.0

    def __init__(self, detector=None, default_k=None):
        self._detector = detector
        self._default_k = default_k

    def _det(self):
        if self._detector is None:
            self._detector = GroundingDinoDetector()
        return self._detector

    def build_steps(self, ctx, params):
        object_name = normalize_prompt(params.get("object_name", "bottle"))
        pitch = math.radians(float(params.get("head_pitch_deg", 30)))
        st = {"frame": None, "det": None}

        def look(c):
            c.body.head.look_at(yaw=0.0, pitch=pitch)
            return StepResult(status="done")

        def capture(c):
            st["frame"] = c.sensor.rgbd(timeout=2.0)
            return StepResult(status="done")

        def detect(c):
            dets = self._det().detect(st["frame"].color, object_name + ".")
            if not dets:
                return StepResult(
                    status="failed",
                    code=ErrorCode.PERCEPTION_OBJECT_NOT_FOUND,
                    retryable=True,
                    data={"object_name": object_name},
                )
            st["det"] = max(dets, key=lambda d: d["score"])
            return StepResult(status="done")

        def locate(c):
            frame, det = st["frame"], st["det"]
            k = getattr(frame, "k", None) or self._default_k
            if k is None:
                return StepResult(
                    status="failed",
                    code=ErrorCode.SYS_CONFIG_ERROR,
                    data={"reason": "缺少相机内参"},
                )
            mask = det.get("mask")
            if mask is not None:
                p_cam, info = estimate_from_mask(mask, frame.depth, k)
            else:
                p_cam, info = estimate_from_box(det["box"], frame.depth, k)
            if p_cam is None:
                return StepResult(
                    status="failed",
                    code=ErrorCode.PERCEPTION_POSE_INVALID,
                    retryable=True,
                    data={"reason": info},
                )
            try:
                p_base = c.sensor.transform_to_base(p_cam)
            except NotImplementedError as exc:
                return StepResult(
                    status="failed",
                    code=ErrorCode.PERCEPTION_POSE_INVALID,
                    data={"reason": f"cam->base 不可用: {exc}"},
                )
            pose = [float(x) for x in p_base]
            if not in_valid_range(pose):
                return StepResult(
                    status="failed",
                    code=ErrorCode.PERCEPTION_POSE_INVALID,
                    data={"obj_pose": pose},
                )
            return StepResult(
                status="done",
                data={
                    "object_name": object_name,
                    "obj_pose": pose,
                    "label": det.get("label", object_name),
                    "score": det.get("score"),
                },
            )

        return [
            Step(name="低头", kind="motion", weight=1, run=look),
            Step(name="取帧", kind="perception", weight=1, tick_budget=2.0, run=capture),
            Step(name="检测", kind="perception", weight=3, tick_budget=2.0,
                 retries=1, run=detect),
            Step(name="定位", kind="perception", weight=2, run=locate),
        ]
