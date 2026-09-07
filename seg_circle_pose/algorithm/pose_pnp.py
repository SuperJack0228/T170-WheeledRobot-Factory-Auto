"""算法 0：分割轮廓 → 椭圆圆周 PnP → 4×4 位姿。"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

MIN_PNP_DEPTH_M = 0.02  # m
MIN_NORMAL_VIEW_DOT = 0.05
MIN_ELLIPSE_AXIS_PX = 1.0  # px，图像几何
DEFAULT_REPROJ_THRESH = 6.0  # px
DEFAULT_MAX_REPROJ_MEAN = 10.0  # px


@dataclass
class EllipseFit:
    center: tuple[float, float]
    axes: tuple[float, float]
    angle_deg: float


@dataclass
class PnpPoseResult:
    success: bool
    pose_4x4: np.ndarray | None
    ellipse: EllipseFit | None = None
    message: str = ""


def rvec_tvec_to_matrix4(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T


def fit_ellipse(contour: np.ndarray, min_axis_px: float = MIN_ELLIPSE_AXIS_PX) -> EllipseFit | None:
    if contour.shape[0] < 5:
        return None
    ((cx, cy), (ma, mi), ang) = cv2.fitEllipse(contour.astype(np.float32))
    if ma < min_axis_px or mi < min_axis_px:
        return None
    return EllipseFit(center=(float(cx), float(cy)), axes=(float(ma), float(mi)), angle_deg=float(ang))


def _ellipse_point(ell: EllipseFit, t: float) -> np.ndarray:
    cx, cy = ell.center
    a, b = ell.axes[0] / 2.0, ell.axes[1] / 2.0
    rad = np.deg2rad(ell.angle_deg)
    ct, st = np.cos(t), np.sin(t)
    cos_r, sin_r = np.cos(rad), np.sin(rad)
    x = cx + a * ct * cos_r - b * st * sin_r
    y = cy + a * ct * sin_r + b * st * cos_r
    return np.array([x, y], dtype=np.float64)


def _sample_ellipse(ell: EllipseFit, n: int, phase: float) -> np.ndarray:
    ts = np.linspace(0, 2 * np.pi, n, endpoint=False) + phase
    return np.stack([_ellipse_point(ell, t) for t in ts], axis=0)


def _circle_model_3d(radius_m: float, n: int, phase: float) -> np.ndarray:
    ts = np.linspace(0, 2 * np.pi, n, endpoint=False) + phase
    return np.stack([radius_m * np.cos(ts), radius_m * np.sin(ts), np.zeros(n)], axis=1).astype(np.float64)


def _canonicalize_z_positive(rvec: np.ndarray, normal: np.ndarray) -> np.ndarray:
    if float(normal[2]) >= 0.0:
        return rvec
    r_mat, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    rx180 = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]], dtype=np.float64)
    r_new, _ = cv2.Rodrigues(r_mat @ rx180)
    return r_new.flatten()


def estimate_pose_pnp(
    contour: np.ndarray,
    radius_m: float,
    K: np.ndarray,
    dist: np.ndarray,
    n_samples: int = 12,
    reproj_thresh: float = DEFAULT_REPROJ_THRESH,
    max_reproj_mean: float = DEFAULT_MAX_REPROJ_MEAN,
    min_depth_m: float = MIN_PNP_DEPTH_M,
    min_normal_view_dot: float = MIN_NORMAL_VIEW_DOT,
) -> PnpPoseResult:
    ell = fit_ellipse(contour)
    if ell is None:
        return PnpPoseResult(False, None, None, "ellipse fit failed")

    best_rvec = best_tvec = None
    best_mean = 1e9

    for phase in (0.0, 0.5 * np.pi, np.pi, 1.5 * np.pi):
        img_pts = _sample_ellipse(ell, n_samples, phase)
        obj_pts = _circle_model_3d(radius_m, n_samples, phase)

        ok, rvec, tvec, _ = cv2.solvePnPRansac(
            obj_pts,
            img_pts.astype(np.float64),
            K,
            dist,
            iterationsCount=300,
            reprojectionError=reproj_thresh,
            confidence=0.95,
            flags=cv2.SOLVEPNP_SQPNP,
        )
        if not ok or rvec is None:
            continue

        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
        err = np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)
        mean_e = float(err.mean())

        r_mat, _ = cv2.Rodrigues(rvec)
        normal = (r_mat @ np.array([0.0, 0.0, 1.0])).reshape(3)
        center_cam = tvec.reshape(3)
        if center_cam[2] <= min_depth_m:
            continue
        view = -center_cam / (np.linalg.norm(center_cam) + 1e-9)
        if abs(float(np.dot(normal, view))) < min_normal_view_dot:
            continue

        if mean_e < best_mean:
            best_mean = mean_e
            best_rvec = _canonicalize_z_positive(rvec.flatten(), normal)
            best_tvec = tvec.flatten()

    if best_rvec is None:
        return PnpPoseResult(False, None, ell, "pnp failed")
    if best_mean > max_reproj_mean:
        return PnpPoseResult(False, None, ell, f"reproj too large {best_mean:.2f}")

    return PnpPoseResult(
        True,
        rvec_tvec_to_matrix4(best_rvec, best_tvec),
        ell,
        "",
    )
