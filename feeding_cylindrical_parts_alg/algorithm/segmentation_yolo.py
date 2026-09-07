"""YOLO 实例分割，输出 mask / 轮廓及类别、置信度。

与 feeding_table_alg3/detection.py 对齐：
  - resolve_imgsz：720p 用 1280 推理，避免默认 640 letterbox 导致 mask 锯齿
  - retina_masks：原图分辨率 mask
  - ops.scale_masks：letterbox mask 映射回原图（勿 cv2.resize）
  - contour_from_mask：轮廓从还原后的二值 mask 提取，与可视化一致
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
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


def resolve_imgsz(bgr: np.ndarray, imgsz: int | list[int] | tuple[int, ...]) -> int | list[int]:
    """
    imgsz<=0 / 'auto' → use max(H,W) so 1280×720 runs at ~1280 instead of default 640.
    At imgsz=640 a 1280×720 frame is letterboxed to ~640×360, then masks are upscaled → jagged edges.
    """
    if isinstance(imgsz, (list, tuple)):
        return list(imgsz)
    if imgsz <= 0:
        h, w = bgr.shape[:2]
        return max(h, w)
    return int(imgsz)


def masks_to_orig_binary(masks, orig_shape: tuple[int, int]) -> np.ndarray:
    """
    Letterbox / native mask tensor → original image (N,H,W) binary mask.

    Uses ops.scale_masks (bilinear + crop padding), same as feeding_table_alg3.
    With retina_masks=True, masks.data is often already (N, H0, W0) — skip rescale.
    """
    m = masks.float() if isinstance(masks, torch.Tensor) else torch.from_numpy(np.asarray(masks)).float()
    oh, ow = int(orig_shape[0]), int(orig_shape[1])
    if m.ndim == 2:
        m = m.unsqueeze(0)
    if m.shape[-2:] != (oh, ow):
        m = ops.scale_masks(m[None], (oh, ow))[0]
    return (m > 0.5).cpu().numpy().astype(np.uint8)


def contour_from_mask(mask: np.ndarray, min_points: int = 5) -> np.ndarray | None:
    """Largest external contour from a (H,W) 0/1 mask in original image coords."""
    mask_u8 = (mask > 0).astype(np.uint8)
    contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    if cnt.shape[0] < min_points:
        return None
    return cnt.reshape(-1, 2).astype(np.float32)


def _instances_from_yolo_result(
    yolo_result,
) -> list[tuple[np.ndarray, np.ndarray | None, int, float]]:
    """(mask, contour, class_id, confidence) in original image coordinates."""
    if yolo_result.masks is None or yolo_result.masks.data is None:
        return []

    orig_shape = yolo_result.orig_shape
    bin_masks = masks_to_orig_binary(yolo_result.masks.data, orig_shape)
    n = len(bin_masks)

    if yolo_result.boxes is not None:
        cls = yolo_result.boxes.cls.cpu().numpy().astype(int)
        conf = yolo_result.boxes.conf.cpu().numpy()
    else:
        cls = np.zeros(n, dtype=int)
        conf = np.ones(n, dtype=np.float32)

    out: list[tuple[np.ndarray, np.ndarray | None, int, float]] = []
    for i in range(n):
        mask = bin_masks[i]
        contour = contour_from_mask(mask)
        cid = int(cls[i]) if i < len(cls) else 0
        score = float(conf[i]) if i < len(conf) else 0.0
        out.append((mask, contour, cid, score))
    return out


class YoloSegmenter:
    def __init__(
        self,
        model_path: str,
        conf: float = 0.25,
        iou: float = 0.7,
        classes: list[int] | None = None,
        imgsz: int | list[int] = 1280,
        retina_masks: bool = True,
    ) -> None:
        self._model = YOLO(model_path)
        self.conf = conf
        self.iou = iou
        self.classes = classes
        self.imgsz = imgsz
        self.retina_masks = retina_masks

    def predict(self, bgr: np.ndarray) -> list[SegInstance]:
        infer_sz = resolve_imgsz(bgr, self.imgsz)
        y = self._model.predict(
            source=bgr,
            conf=self.conf,
            iou=self.iou,
            classes=self.classes,
            imgsz=infer_sz,
            retina_masks=self.retina_masks,
            verbose=False,
        )[0]

        instances: list[SegInstance] = []
        for mask, contour, class_id, confidence in _instances_from_yolo_result(y):
            instances.append(
                SegInstance(
                    mask=mask,
                    contour=contour,
                    class_id=class_id,
                    confidence=confidence,
                )
            )
        return instances
