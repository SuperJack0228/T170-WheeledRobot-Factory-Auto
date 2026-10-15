"""YOLO 检测 /（可选）实例分割前端。

主路径（detect 权重如 best1015）：只出框 + 类别，不再生成伪 mask。
若权重带 masks，仍填充 mask/contour 供旧算法 0/1 使用。
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
    class_id: int
    confidence: float
    xyxy: np.ndarray
    """(4,) float32：x1,y1,x2,y2 原图像素。"""
    mask: np.ndarray | None = None
    """(H, W) uint8 0/1；仅分割权重有。"""
    contour: np.ndarray | None = None
    """轮廓多边形 (N, 2)；仅分割权重有。"""
    from_detect: bool = True
    """True：检测模型（无原生 mask），引擎走算法 2 框中心。"""


def resolve_imgsz(bgr: np.ndarray, imgsz: int | list[int] | tuple[int, ...]) -> int | list[int]:
    if isinstance(imgsz, (list, tuple)):
        return list(imgsz)
    if imgsz <= 0:
        h, w = bgr.shape[:2]
        return max(h, w)
    return int(imgsz)


def masks_to_orig_binary(masks, orig_shape: tuple[int, int]) -> np.ndarray:
    m = masks.float() if isinstance(masks, torch.Tensor) else torch.from_numpy(np.asarray(masks)).float()
    oh, ow = int(orig_shape[0]), int(orig_shape[1])
    if m.ndim == 2:
        m = m.unsqueeze(0)
    if m.shape[-2:] != (oh, ow):
        m = ops.scale_masks(m[None], (oh, ow))[0]
    return (m > 0.5).cpu().numpy().astype(np.uint8)


def contour_from_mask(mask: np.ndarray, min_points: int = 5) -> np.ndarray | None:
    mask_u8 = (mask > 0).astype(np.uint8)
    contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    if cnt.shape[0] < min_points:
        return None
    return cnt.reshape(-1, 2).astype(np.float32)


def _instances_from_yolo_result(yolo_result) -> list[SegInstance]:
    if yolo_result.boxes is None or len(yolo_result.boxes) == 0:
        return []

    xyxy = yolo_result.boxes.xyxy.cpu().numpy().astype(np.float32)
    cls = yolo_result.boxes.cls.cpu().numpy().astype(int)
    conf = yolo_result.boxes.conf.cpu().numpy().astype(np.float32)
    n = len(xyxy)

    has_masks = (
        yolo_result.masks is not None
        and yolo_result.masks.data is not None
        and len(yolo_result.masks.data) > 0
    )
    bin_masks = None
    if has_masks:
        bin_masks = masks_to_orig_binary(yolo_result.masks.data, yolo_result.orig_shape)

    out: list[SegInstance] = []
    for i in range(n):
        cid = int(cls[i]) if i < len(cls) else 0
        score = float(conf[i]) if i < len(conf) else 0.0
        box = xyxy[i]
        if has_masks and bin_masks is not None and i < len(bin_masks):
            mask = bin_masks[i]
            out.append(
                SegInstance(
                    class_id=cid,
                    confidence=score,
                    xyxy=box,
                    mask=mask,
                    contour=contour_from_mask(mask),
                    from_detect=False,
                )
            )
        else:
            out.append(
                SegInstance(
                    class_id=cid,
                    confidence=score,
                    xyxy=box,
                    mask=None,
                    contour=None,
                    from_detect=True,
                )
            )
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
        return _instances_from_yolo_result(y)
