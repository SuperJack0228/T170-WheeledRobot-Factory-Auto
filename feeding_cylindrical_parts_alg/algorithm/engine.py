"""算法统一入口：YOLO 检测/分割 + 按 algorithm_id 估计位姿。"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from algorithm.config_loader import (
    PoseParams,
    load_pose_params,
)
from algorithm.pose_box import estimate_pose_box_center
from algorithm.pose_centroid import estimate_pose_centroid
from algorithm.pose_pnp import estimate_pose_pnp
from algorithm.segmentation_yolo import YoloSegmenter
from algorithm.types import (
    ALGO_BOX,
    ALGO_CENTROID,
    ALGO_IDS,
    ALGO_PNP,
    AlgorithmInput,
    AlgorithmOutput,
    TargetPose,
)

DEFAULT_RADIUS_M = 0.025


class CirclePoseEngine:
    """
    YOLO 位姿/关联点引擎。

    algorithm_id:
      0 — 分割 + 椭圆 PnP（需原生 mask）
      1 — 分割 + mask 深度重心
      2 — 检测框中心 + 深度粗点（detect 权重；料盘关联用）

    detect 权重（from_detect=True）一律走算法 2，忽略帧上的 0/1 覆盖。
    """

    def __init__(self, params: PoseParams | str | Path | None = None) -> None:
        if params is None:
            self.params = load_pose_params()
        elif isinstance(params, PoseParams):
            self.params = params
        else:
            self.params = load_pose_params(params)

        p = self.params
        model_file = p.segmentation.model_path
        if not model_file.is_file():
            raise FileNotFoundError(
                f"YOLO 模型不存在: {model_file}\n"
                f"请检查 config 中 segmentation.model 或放置到 models/best1015.pt"
            )

        self._segmenter = YoloSegmenter(
            str(model_file),
            conf=p.segmentation.conf,
            iou=p.segmentation.iou,
            classes=p.enabled_class_ids(),
            imgsz=p.segmentation.imgsz,
            retina_masks=p.segmentation.retina_masks,
        )
        self._model_path = str(model_file)

    def warmup(self, height: int = 720, width: int = 1280) -> str:
        """空图推理一次。模型对象保持在 self._segmenter，拍照时不再 YOLO()。"""
        self._segmenter.warmup(height, width)
        return self._model_path

    def _resolve_algorithm_id(self, class_id: int, frame_override: int | None, from_detect: bool) -> int:
        if from_detect:
            return ALGO_BOX
        if frame_override is not None:
            if frame_override not in ALGO_IDS:
                raise ValueError(f"unsupported algorithm_id={frame_override}, use {ALGO_IDS}")
            return frame_override
        return self.params.algorithm_id_for(class_id)

    def _estimate_one(
        self,
        inst,
        algo_id: int,
        K: np.ndarray,
        dist: np.ndarray,
        depth_m: np.ndarray | None,
    ) -> TargetPose:
        spec = self.params.get_class(inst.class_id)
        class_name = spec.name if spec else f"class_{inst.class_id}"
        radius_m = self.params.radius_m_for(inst.class_id)

        base_kw = dict(
            class_id=inst.class_id,
            confidence=inst.confidence,
            mask=inst.mask,
            xyxy=np.asarray(inst.xyxy, dtype=np.float32).reshape(4),
            class_name=class_name,
            radius_m=radius_m,
            algorithm_id=algo_id,
        )

        if spec is not None and not spec.enabled:
            return TargetPose(
                np.eye(4), success=False, message="class disabled in config", **base_kw
            )

        if algo_id == ALGO_BOX:
            res = estimate_pose_box_center(inst.xyxy, depth_m, K)
            return TargetPose(
                res.pose_4x4 if res.pose_4x4 is not None else np.eye(4),
                success=res.success,
                message=res.message,
                **base_kw,
            )

        if algo_id == ALGO_PNP:
            if inst.contour is None:
                return TargetPose(
                    np.eye(4), success=False, message="contour too short", **base_kw
                )
            pnp_cfg = self.params.pnp_for(inst.class_id)
            res = estimate_pose_pnp(
                inst.contour,
                radius_m,
                K,
                dist,
                n_samples=pnp_cfg.n_samples,
                reproj_thresh=pnp_cfg.reproj_thresh,
                max_reproj_mean=pnp_cfg.max_reproj_mean,
                min_depth_m=pnp_cfg.min_depth_m,
                min_normal_view_dot=pnp_cfg.min_normal_view_dot,
                min_ellipse_area_px=pnp_cfg.min_ellipse_area_px,
            )
            return TargetPose(
                res.pose_4x4 if res.pose_4x4 is not None else np.eye(4),
                success=res.success,
                message=res.message,
                ellipse=res.ellipse,
                **base_kw,
            )

        if depth_m is None:
            return TargetPose(
                np.eye(4),
                success=False,
                message="centroid requires depth",
                **base_kw,
            )
        if inst.mask is None:
            return TargetPose(
                np.eye(4),
                success=False,
                message="centroid requires mask",
                **base_kw,
            )
        cen_cfg = self.params.centroid_for(inst.class_id)
        res = estimate_pose_centroid(
            inst.mask,
            depth_m,
            K,
            mask_erode_kernel=cen_cfg.mask_erode_kernel,
            min_depth_m=cen_cfg.min_depth_m,
            max_depth_m=cen_cfg.max_depth_m,
            min_points=cen_cfg.min_points,
        )
        return TargetPose(
            res.pose_4x4 if res.pose_4x4 is not None else np.eye(4),
            success=res.success,
            message=res.message,
            **base_kw,
        )

    def run(self, frame: AlgorithmInput) -> AlgorithmOutput:
        K = np.asarray(frame.K, dtype=np.float64)
        dist = np.asarray(frame.dist_coeffs, dtype=np.float64).reshape(-1)
        instances = self._segmenter.predict(frame.rgb)

        depth_m = np.asarray(frame.depth, dtype=np.float32) if frame.depth is not None else None
        targets: list[TargetPose] = []
        for inst in instances:
            if not self.params.is_class_enabled(inst.class_id):
                continue
            algo_id = self._resolve_algorithm_id(
                inst.class_id, frame.algorithm_id, inst.from_detect
            )
            if algo_id == ALGO_CENTROID and depth_m is None:
                raise ValueError("algorithm_id=1 (centroid) requires depth aligned to rgb")
            pose = self._estimate_one(inst, algo_id, K, dist, depth_m)
            if not pose.success and (
                pose.message.startswith("ellipse area too small")
                or pose.message.startswith("depth out of range")
            ):
                continue
            pose.mask = inst.mask
            pose.xyxy = np.asarray(inst.xyxy, dtype=np.float32).reshape(4)
            targets.append(pose)

        return AlgorithmOutput(targets=targets)
