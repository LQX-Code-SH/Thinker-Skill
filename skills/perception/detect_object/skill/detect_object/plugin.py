"""detect_object：取帧 → 检测 → 3D 定位 → base 系位置（只读，不下发动作）。"""
from __future__ import annotations

import os
from pathlib import Path

from thinker_skill_sdk import ErrorCode, Step, StepResult

from .detector import DEFAULT_MODEL, GroundingDinoDetector
from .geometry import estimate_from_box, estimate_from_mask
from .spatial import in_valid_range, normalize_prompt


def resolve_model_path(raw, ctx):
    """解析模型目录：绝对路径直接用；相对路径依次尝试

    1. 技能根目录（部署包里 ``<ver>/weights/...``）
    2. 当前工作目录（源码树仓库根）
    3. ``THINKER_WEIGHTS_DIR`` + 目录名
    """
    p = Path(raw or DEFAULT_MODEL)
    if p.is_absolute():
        return str(p)
    candidates = []
    skill_dir = getattr(ctx, "skill_dir", None)
    if skill_dir:
        candidates.append(Path(skill_dir) / p)
    candidates.append(Path.cwd() / p)
    env = os.environ.get("THINKER_WEIGHTS_DIR")
    if env:
        candidates.append(Path(env) / p.name)
    for candidate in candidates:
        if candidate.is_dir():
            return str(candidate)
    return str(p)


class DetectObject:
    name = "detect_object"
    # 只读技能：只取相机数据，不下发头部/身体动作，不占身体租约
    requires = ["sensor.rgbd"]
    postcondition = {}
    timeout = 60.0

    def __init__(self, detector=None, default_k=None):
        self._detector = detector
        self._default_k = default_k

    def _det(self, model_path=None):
        if self._detector is None:
            self._detector = GroundingDinoDetector(
                model_path=model_path or DEFAULT_MODEL)
        return self._detector

    def build_steps(self, ctx, params):
        cfg = dict(ctx.cfg or {})
        object_name = normalize_prompt(params.get("object_name", "bottle"))
        model_path = resolve_model_path(cfg.get("model_path"), ctx)
        st = {"frame": None, "det": None}

        def capture(c):
            st["frame"] = c.sensor.rgbd(timeout=2.0)
            return StepResult(status="done")

        def detect(c):
            dets = self._det(model_path).detect(
                st["frame"].color, object_name + ".")
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
            Step(name="取帧", kind="perception", weight=1, tick_budget=2.0, run=capture),
            Step(name="检测", kind="perception", weight=3, tick_budget=2.0,
                 retries=1, run=detect),
            Step(name="定位", kind="perception", weight=2, run=locate),
        ]
