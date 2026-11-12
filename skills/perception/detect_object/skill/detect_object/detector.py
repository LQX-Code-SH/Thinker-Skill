"""GroundingDINO 开集检测（延迟加载模型，输入 BGR）。"""
from __future__ import annotations

import numpy as np

DEFAULT_MODEL = "weights/grounding-dino-tiny"


class GroundingDinoDetector:
    def __init__(self, model_path=DEFAULT_MODEL, threshold=0.35,
                 text_threshold=0.35, device="cuda"):
        self._model_path = model_path
        self._threshold = float(threshold)
        self._text_threshold = float(text_threshold)
        self._device = device
        self._model = None
        self._processor = None

    def _ensure(self):
        if self._model is not None:
            return
        from transformers import (
            AutoModelForZeroShotObjectDetection,
            AutoProcessor,
        )
        self._processor = AutoProcessor.from_pretrained(self._model_path)
        self._model = AutoModelForZeroShotObjectDetection.from_pretrained(
            self._model_path
        ).to(self._device)

    def detect(self, image_bgr, prompt):
        """返回 ``[{"box": [x1,y1,x2,y2], "score": float, "label": str}]``。"""
        self._ensure()
        import torch

        # 负 stride 数组 torchvision 不接受，必须转成连续内存
        rgb = np.ascontiguousarray(image_bgr[..., ::-1])
        inputs = self._processor(
            images=rgb, text=prompt, return_tensors="pt"
        ).to(self._device)
        with torch.no_grad():
            outputs = self._model(**inputs)
        result = self._processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            threshold=self._threshold,
            text_threshold=self._text_threshold,
            target_sizes=[rgb.shape[:2]],
        )[0]
        return [
            {
                "box": [float(v) for v in box.tolist()],
                "score": float(score),
                "label": str(label),
            }
            for box, score, label in zip(
                result["boxes"], result["scores"], result["labels"]
            )
        ]
