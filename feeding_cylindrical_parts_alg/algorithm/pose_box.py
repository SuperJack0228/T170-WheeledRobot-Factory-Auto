"""算法 2：检测框中心 + 深度 → 相机系平移（R=I）。

供料盘孔↔类别关联、手相机粗匹配用；不是抓取目标位姿来源。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

MIN_DEPTH_M = 0.05
MAX_DEPTH_M = 2.0
PATCH_HALF_PX = 8
MIN_DEPTH_POINTS = 4
NOMINAL_Z_M = 0.80  # 无深度时沿光心射线的占位深度（仅关联用）


@dataclass
class BoxPoseResult:
    success: bool
    pose_4x4: np.ndarray | None
    message: str = ""
    uv: tuple[float, float] | None = None


def estimate_pose_box_center(
    xyxy: np.ndarray,
    depth_m: np.ndarray | None,
    K: np.ndarray,
    patch_half_px: int = PATCH_HALF_PX,
    min_depth_m: float = MIN_DEPTH_M,
    max_depth_m: float = MAX_DEPTH_M,
    min_points: int = MIN_DEPTH_POINTS,
    nominal_z_m: float = NOMINAL_Z_M,
) -> BoxPoseResult:
    box = np.asarray(xyxy, dtype=np.float64).reshape(4)
    u = 0.5 * (box[0] + box[2])
    v = 0.5 * (box[1] + box[3])
    if not (np.isfinite(u) and np.isfinite(v)):
        return BoxPoseResult(False, None, "box center invalid")

    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    if fx <= 1e-9 or fy <= 1e-9:
        return BoxPoseResult(False, None, "bad intrinsics")

    z: float | None = None
    msg = "box_center"
    if depth_m is not None:
        depth = np.asarray(depth_m)
        h, w = depth.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        x0 = max(0, ui - patch_half_px)
        x1 = min(w, ui + patch_half_px + 1)
        y0 = max(0, vi - patch_half_px)
        y1 = min(h, vi + patch_half_px + 1)
        if x1 > x0 and y1 > y0:
            patch = depth[y0:y1, x0:x1].astype(np.float64)
            valid = np.isfinite(patch) & (patch >= min_depth_m) & (patch <= max_depth_m)
            vals = patch[valid]
            if vals.size >= min_points:
                z = float(np.median(vals))
                msg = "box_center_depth"
            elif vals.size > 0:
                z = float(np.median(vals))
                msg = "box_center_depth_sparse"

    if z is None:
        z = float(nominal_z_m)
        msg = "box_center_nominal_z"

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    T = np.eye(4, dtype=np.float64)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return BoxPoseResult(True, T, msg, uv=(float(u), float(v)))
