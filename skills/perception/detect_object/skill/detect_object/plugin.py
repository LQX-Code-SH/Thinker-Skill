"""detect_object：取帧 → 检测 → 3D 定位 → base 系位置（只读，不下发动作）。"""
from __future__ import annotations

import os
from pathlib import Path

from thinker_skill_sdk import ErrorCode, Step, StepResult

from .detector import DEFAULT_MODEL, GroundingDinoDetector
from .geometry import (estimate_from_box, estimate_from_cloud,
                       estimate_from_mask, estimate_from_plane)
from .sam2 import DEFAULT_MODEL as DEFAULT_SAM2_MODEL, Sam2Segmenter
from .spatial import in_valid_range, normalize_prompt

_MISS = object()          # _radius_for 内部哨兵：表示"无法解析"


def refine_mask(mask, box, min_area=50):
    """取 mask 的最大连通域，并把 2D 框收紧到它的包围盒。

    DINO 的框是按"物体语义"给的，边缘常比真实轮廓松；SAM2 也会偶尔带上
    与物体相连的背景碎片（实测水瓶的 extent 因此涨到 0.164 m，真实直径
    0.06 m）。取最大连通域可去掉碎片，再用它的包围盒替代原框，框高/框宽
    就与物体轮廓一致，plane 方法反算物理高度也更准。
    """
    try:
        import cv2
        import numpy as np
    except ImportError:
        return mask, list(box)

    binary = (np.asarray(mask) > 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    if count <= 1:
        return mask, list(box)
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if int(stats[idx, cv2.CC_STAT_AREA]) < int(min_area):
        return mask, list(box)
    x, y, w, h = (int(v) for v in stats[idx, :4])
    return labels == idx, [float(x), float(y), float(x + w), float(y + h)]


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

    def __init__(self, detector=None, default_k=None, segmenter=None):
        self._detector = detector
        self._default_k = default_k
        self._segmenter = segmenter

    def _det(self, model_path=None, threshold=0.35, text_threshold=0.35):
        if self._detector is None:
            self._detector = GroundingDinoDetector(
                model_path=model_path or DEFAULT_MODEL,
                threshold=threshold, text_threshold=text_threshold)
        return self._detector

    def _seg(self, model_path=None, min_area=50):
        if self._segmenter is None:
            self._segmenter = Sam2Segmenter(
                model_path=model_path or DEFAULT_SAM2_MODEL,
                min_area=min_area)
        return self._segmenter

    @staticmethod
    def _radius_for(locate_cfg, object_name):
        """补偿半径：``auto`` → None（按点云横向尺寸算），数值 → 固定值。

        优先级：``objects.<类别>.radius`` > ``locate.radius`` > 0.03 兜底。
        """
        cfg = locate_cfg if isinstance(locate_cfg, dict) else {}

        def _parse(value):
            """返回 None（=auto）、float（固定值）或 _MISS（无法识别）。"""
            if isinstance(value, str):
                if value.strip().lower() in ("auto", "surface", "physical"):
                    return None
                try:
                    return float(value)
                except ValueError:
                    return _MISS
            if isinstance(value, (int, float)):
                return float(value)
            return _MISS

        entry = (cfg.get("objects") or {}).get(object_name) or {}
        if isinstance(entry, dict) and entry.get("radius") is not None:
            parsed = _parse(entry["radius"])
            if parsed is not _MISS:
                return parsed
        for key in ("radius", "radius_offset"):         # radius_offset 为旧键
            if cfg.get(key) is not None:
                parsed = _parse(cfg[key])
                if parsed is not _MISS:
                    return parsed
        return 0.03

    def build_steps(self, ctx, params):
        cfg = dict(ctx.cfg or {})
        object_name = normalize_prompt(params.get("object_name", "bottle"))
        model_path = resolve_model_path(cfg.get("model_path"), ctx)
        threshold = float(cfg.get("threshold", 0.35))
        text_threshold = float(cfg.get("text_threshold", 0.35))
        # 可选：SAM2 分割（默认开，失败自动回落框中心单点）
        seg_cfg = dict(cfg.get("segment") or {})
        seg_backend = str(seg_cfg.get("backend", "sam2")).lower()
        seg_path = resolve_model_path(
            seg_cfg.get("model_path") or DEFAULT_SAM2_MODEL, ctx)
        seg_min_area = int(seg_cfg.get("min_area", 50))
        # 3D 定位方式：cloud_median（点云中位数）| point（框中心单点）
        loc_cfg = dict(cfg.get("locate") or {})
        locate_method = str(loc_cfg.get("method", "cloud_median")).lower()
        radius = self._radius_for(loc_cfg, object_name)
        # plane 方法需要已知的支撑平面高度（base 系，米）；未配置时自动回落
        plane_z = loc_cfg.get("plane_z")
        plane_z = float(plane_z) if isinstance(plane_z, (int, float)) else None
        st = {"frame": None, "det": None}

        def capture(c):
            st["frame"] = c.sensor.rgbd(timeout=2.0)
            return StepResult(status="done")

        def detect(c):
            dets = self._det(model_path, threshold, text_threshold).detect(
                st["frame"].color, object_name + ".")
            if not dets:
                return StepResult(
                    status="failed",
                    code=ErrorCode.PERCEPTION_OBJECT_NOT_FOUND,
                    retryable=True,
                    data={"object_name": object_name},
                )
            det = max(dets, key=lambda d: d["score"])
            if seg_backend == "sam2":
                mask = self._seg(seg_path, seg_min_area).segment(
                    st["frame"].color, det["box"])
                if mask is not None:
                    clean, refined = refine_mask(mask, det["box"])
                    det["mask"] = clean
                    det["box_raw"] = [float(v) for v in det["box"]]
                    det["box"] = refined
            st["det"] = det
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
            angles = getattr(frame, "angles", None)
            p_base = None
            info = {}
            # plane：物体立在已知高度的平面上，用"框高 → 物理高度"直接定 z，
            # 不依赖可见面的深度补偿（单视角半面误差的正解之一）。
            if mask is not None and locate_method == "plane" and plane_z is not None:
                try:
                    p_base, info = estimate_from_plane(
                        mask, frame.depth, k, box=det.get("box"),
                        plane_z=plane_z,
                        to_base=lambda pt: c.sensor.transform_to_base(pt, angles))
                except NotImplementedError as exc:
                    return StepResult(
                        status="failed",
                        code=ErrorCode.PERCEPTION_POSE_INVALID,
                        data={"reason": f"cam->base 不可用: {exc}"},
                    )
                method_used = "plane"
            else:
                if mask is not None and locate_method == "cloud_median":
                    p_cam, info = estimate_from_cloud(
                        mask, frame.depth, k, radius=radius)
                    method_used = "cloud_median"
                elif mask is not None:
                    p_cam, info = estimate_from_mask(mask, frame.depth, k)
                    method_used = "mask"
                else:
                    p_cam, info = estimate_from_box(det["box"], frame.depth, k)
                    method_used = "box"
                if p_cam is not None:
                    try:
                        # 用取帧时刻的关节角，避免与 locate 时刻不一致
                        p_base = c.sensor.transform_to_base(p_cam, angles)
                    except NotImplementedError as exc:
                        return StepResult(
                            status="failed",
                            code=ErrorCode.PERCEPTION_POSE_INVALID,
                            data={"reason": f"cam->base 不可用: {exc}"},
                        )
            if p_base is None:
                return StepResult(
                    status="failed",
                    code=ErrorCode.PERCEPTION_POSE_INVALID,
                    retryable=True,
                    data={"reason": info if isinstance(info, str) else "定位失败"},
                )
            pose = [float(x) for x in p_base]
            if not in_valid_range(pose):
                return StepResult(
                    status="failed",
                    code=ErrorCode.PERCEPTION_POSE_INVALID,
                    data={"obj_pose": pose},
                )
            payload = {
                "object_name": object_name,
                "obj_pose": pose,
                "label": det.get("label", object_name),
                "score": det.get("score"),
                "box": [float(v) for v in det.get("box", [])],
                "stamp": frame.stamp,
                # 以下是可选信息，旧消费方忽略即可
                "locate": method_used,
                "segmentation": "sam2" if mask is not None else "none",
                "mask_area": int(mask.sum()) if mask is not None else None,
                "extent": (info.get("extent")
                           if isinstance(info, dict) else None),
                "points": (info.get("points")
                           if isinstance(info, dict) else None),
                "radius": (info.get("radius")
                           if isinstance(info, dict) else None),
                "outliers": (info.get("outliers")
                             if isinstance(info, dict) else None),
            }
            # 写入语义状态，供下游技能（抓取/放置）直接复用
            c.state.set("last_detection", payload)
            return StepResult(status="done", data=payload)

        return [
            Step(name="取帧", kind="perception", weight=1, tick_budget=2.0, run=capture),
            Step(name="检测", kind="perception", weight=3, tick_budget=2.0,
                 retries=1, run=detect),
            Step(name="定位", kind="perception", weight=2, run=locate),
        ]
