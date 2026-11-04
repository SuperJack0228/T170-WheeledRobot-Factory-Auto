"""算法 1：分割 mask + 深度 → 重心，姿态 4×4 为单位平移。"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

MASK_ERODE_KERNEL = (9, 9)
MIN_DEPTH_M = 0.05  # m
MIN_CENTROID_POINTS = 10


@dataclass
class CentroidPoseResult:
    success: bool
    pose_4x4: np.ndarray | None
    message: str = ""


def erode_mask_edges(bin_mask: np.ndarray, kernel_size: tuple[int, int] = MASK_ERODE_KERNEL) -> np.ndarray:
    kernel = np.ones(kernel_size, np.uint8)
    return cv2.morphologyEx(bin_mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)


def estimate_pose_centroid(
    bin_mask: np.ndarray,
    depth_m: np.ndarray,
    K: np.ndarray,
    mask_erode_kernel: tuple[int, int] = MASK_ERODE_KERNEL,
    min_depth_m: float = MIN_DEPTH_M,
    max_depth_m: float = 0.0,
    min_points: int = MIN_CENTROID_POINTS,
) -> CentroidPoseResult:
    opened = erode_mask_edges(bin_mask, kernel_size=mask_erode_kernel)
    v, u = np.where(opened > 0)
    if len(u) == 0:
        return CentroidPoseResult(False, None, "mask empty after erosion")

    depths = depth_m[v, u].astype(np.float64)
    finite = np.isfinite(depths)
    valid = finite & (depths >= min_depth_m)
    if max_depth_m > 0.0:
        too_far = valid & (depths > max_depth_m)
        n_far = int(too_far.sum())
        valid = valid & (depths <= max_depth_m)
        if int(valid.sum()) < min_points and n_far > 0:
            z_far = float(np.median(depths[too_far]))
            return CentroidPoseResult(
                False,
                None,
                f"depth out of range (z={z_far:.3f}m > max={max_depth_m:.3f}m, {n_far} pts)",
            )
    if int(valid.sum()) < min_points:
        return CentroidPoseResult(
            False, None, f"too few valid depth points ({int(valid.sum())})"
        )

    u_v = u[valid].astype(np.float64)
    v_v = v[valid].astype(np.float64)
    z = depths[valid]
    fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
    x = (u_v - cx) * z / fx
    y = (v_v - cy) * z / fy
    center = np.array([x.mean(), y.mean(), z.mean()], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = center
    return CentroidPoseResult(True, T, "centroid")
