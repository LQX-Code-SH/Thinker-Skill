"""SAM2 box-prompt 分割（延迟加载，输出与原图同尺寸的 bool mask）。

用法：

    seg = Sam2Segmenter("weights/sam2-hiera-tiny")
    mask = seg.segment(image_bgr, [x1, y1, x2, y2])   # H×W bool 或 None

设计约定：

- DINO 给 box，SAM2 按 box 做 prompt，取 iou 最高的候选 mask；
- 任何异常（权重缺失、CUDA 不可用、分割为空）都返回 ``None``，
  由调用方回落到"框中心单点深度"的老路径 —— 主流程不能被可选模块拖死。
"""
from __future__ import annotations

import os

import numpy as np

DEFAULT_MODEL = "weights/sam2-hiera-tiny"


class Sam2Segmenter:
    def __init__(self, model_path=DEFAULT_MODEL, device="cuda",
                 min_area: int = 50):
        self._model_path = str(model_path)
        self._device = device
        self._min_area = int(min_area)
        self._model = None
        self._processor = None

    def _ensure(self):
        if self._model is not None:
            return
        import torch
        from transformers import Sam2Model, Sam2Processor

        self._processor = Sam2Processor.from_pretrained(self._model_path)
        self._model = Sam2Model.from_pretrained(self._model_path)
        self._model = self._model.to(self._device).eval()
        self._torch = torch

    def segment(self, image_bgr, box):
        """box ``[x1, y1, x2, y2]`` → 与原图同尺寸的 bool mask；失败返回 None。"""
        try:
            self._ensure()
            torch = self._torch

            rgb = np.ascontiguousarray(image_bgr[..., ::-1])
            h, w = rgb.shape[:2]
            inputs = self._processor(
                images=rgb,
                input_boxes=[[[float(v) for v in box[:4]]]],
                return_tensors="pt",
            )
            inputs = {
                k: (v.to(self._device) if hasattr(v, "to") else v)
                for k, v in inputs.items()
            }
            with torch.no_grad():
                outputs = self._model(**inputs, multimask_output=True)

            # 直接对低分辨率 logits 上采样，不依赖 processor 的后处理 API
            # （transformers 5.0 里 post_process_masks 的入参名与旧版不同）。
            logits = outputs.pred_masks
            if logits.ndim == 5:            # (B, num, 1, h, w)
                logits = logits[:, :, 0]
            logits = logits.float()
            num = int(logits.shape[1])
            ious = outputs.iou_scores.reshape(-1)
            idx = int(torch.argmax(ious)) if ious.numel() else 0
            idx = min(max(idx, 0), num - 1)              # iou 数与 mask 数可能不一致
            best = logits[0, idx:idx + 1].unsqueeze(0)   # (1, 1, h, w)
            up = torch.nn.functional.interpolate(
                best, size=(h, w), mode="bilinear", align_corners=False)
            mask = (up[0, 0] > 0.0).detach().cpu().numpy()

            if mask.sum() < self._min_area:
                return None
            return mask
        except Exception:  # noqa: BLE001 - 可选模块，失败即回退
            if os.environ.get("THINKER_SAM2_DEBUG"):
                import traceback
                traceback.print_exc()
            return None
