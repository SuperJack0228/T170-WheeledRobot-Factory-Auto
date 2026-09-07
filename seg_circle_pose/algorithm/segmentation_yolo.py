"""YOLO 实例分割，输出 mask / 轮廓及类别、置信度。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.utils import ops


@dataclass
class SegInstance:
    mask: np.ndarray
    """(H, W) uint8, 0/1。"""
    contour: np.ndarray | None
    """轮廓多边形 (N, 2) float32，点数不足时为 None。"""
    class_id: int
    confidence: float


def _masks_to_orig_binary(masks, orig_shape: tuple[int, int]) -> np.ndarray:
    """letterbox 空间掩膜 → 原图 (N,H,W) uint8 0/1，与 Ultralytics results.plot() 一致。"""
    m = masks.float() if isinstance(masks, torch.Tensor) else torch.from_numpy(np.asarray(masks)).float()
    if m.ndim == 2:
        m = m.unsqueeze(0)
    oh, ow = orig_shape[:2]
    if m.shape[-2:] != (oh, ow):
        m = ops.scale_masks(m[None], (oh, ow))[0]
    return (m > 0.5).cpu().numpy().astype(np.uint8)


class YoloSegmenter:
    def __init__(
        self,
        model_path: str,
        conf: float = 0.25,
        iou: float = 0.7,
        classes: list[int] | None = None,
    ) -> None:
        self._model = YOLO(model_path)
        self.conf = conf
        self.iou = iou
        self.classes = classes

    def predict(self, bgr: np.ndarray) -> list[SegInstance]:
        y = self._model.predict(
            source=bgr,
            conf=self.conf,
            iou=self.iou,
            classes=self.classes,
            verbose=False,
        )[0]

        if y.masks is None or y.masks.data is None:
            return []

        n = len(y.masks.data)
        cls = y.boxes.cls.cpu().numpy().astype(int) if y.boxes is not None else np.zeros(n, dtype=int)
        conf = y.boxes.conf.cpu().numpy() if y.boxes is not None else np.ones(n, dtype=np.float32)

        # masks.data 在 letterbox 空间；scale_masks 映射到 orig_shape，勿 cv2.resize
        bin_masks = _masks_to_orig_binary(y.masks.data, y.orig_shape)

        instances: list[SegInstance] = []
        contours_xy = y.masks.xy if y.masks.xy is not None else [None] * n

        for i in range(n):
            mask = bin_masks[i]

            contour = None
            if i < len(contours_xy) and contours_xy[i] is not None:
                poly = np.asarray(contours_xy[i], dtype=np.float32)
                if poly.shape[0] >= 5:
                    contour = poly

            instances.append(
                SegInstance(
                    mask=mask,
                    contour=contour,
                    class_id=int(cls[i]) if i < len(cls) else 0,
                    confidence=float(conf[i]) if i < len(conf) else 0.0,
                )
            )
        return instances
