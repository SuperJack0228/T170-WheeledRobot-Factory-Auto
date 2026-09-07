"""检测结果可视化：mask、PnP 椭圆/位姿轴、重心位置。"""

from __future__ import annotations

import cv2
import numpy as np

from algorithm.pose_pnp import EllipseFit
from algorithm.types import ALGO_PNP, AlgorithmOutput, TargetPose

AXIS_COLORS = ((0, 0, 255), (0, 255, 0), (255, 0, 0))  # X Y Z (BGR)
MASK_COLORS = (
    (255, 128, 0),
    (0, 200, 255),
    (200, 0, 200),
    (0, 255, 128),
    (128, 128, 255),
)


def _mask_color(class_id: int) -> tuple[int, int, int]:
    return MASK_COLORS[class_id % len(MASK_COLORS)]


def pose_to_xyz_rpy_deg(pose_4x4: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    4×4 位姿 → 平移 (m) + 欧拉角 roll-pitch-yaw (deg)，相机系 ZYX 内旋顺序。
    """
    t = pose_4x4[:3, 3].astype(np.float64)
    R = pose_4x4[:3, :3].astype(np.float64)
    pitch = np.arcsin(float(np.clip(-R[2, 0], -1.0, 1.0)))
    cp = np.cos(pitch)
    if abs(cp) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[0, 1], R[1, 1])
        yaw = 0.0
    rpy_deg = np.rad2deg([roll, pitch, yaw])
    return t, rpy_deg


def _put_text_lines(
    img: np.ndarray,
    lines: list[str],
    origin: tuple[int, int],
    color: tuple[int, int, int],
    scale: float = 0.48,
    line_h: int = 18,
) -> None:
    x, y = origin
    for i, line in enumerate(lines):
        cv2.putText(
            img,
            line,
            (x, y + i * line_h),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            1,
            cv2.LINE_AA,
        )


def draw_mask_overlay(
    img: np.ndarray,
    mask: np.ndarray,
    color: tuple[int, int, int],
    alpha: float = 0.35,
) -> None:
    if mask is None or not np.any(mask > 0):
        return
    region = mask > 0
    overlay = img.copy()
    overlay[region] = color
    img[:] = cv2.addWeighted(overlay, alpha, img, 1.0 - alpha, 0)
    cnts, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if cnts:
        cv2.drawContours(img, cnts, -1, color, 2, cv2.LINE_AA)


def draw_ellipse_fit(img: np.ndarray, ell: EllipseFit) -> None:
    box = cv2.boxPoints(
        ((ell.center[0], ell.center[1]), ell.axes, ell.angle_deg)
    ).astype(np.int32)
    cv2.polylines(img, [box], True, (255, 200, 0), 2, cv2.LINE_AA)


def draw_pnp_pose(
    img: np.ndarray,
    pose_4x4: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    radius_m: float,
    show_xy_axes: bool = False,
) -> None:
    R = pose_4x4[:3, :3]
    t = pose_4x4[:3, 3].reshape(3, 1)
    rvec, _ = cv2.Rodrigues(R)

    axis_len = max(float(radius_m) * 1.5, 0.01)
    axes_3d = np.float32(
        [[0, 0, 0], [axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]]
    )
    proj_ax, _ = cv2.projectPoints(axes_3d, rvec, t, K, dist)
    proj_ax = proj_ax.reshape(-1, 2).astype(int)
    origin = tuple(proj_ax[0])
    axis_indices = [1, 2, 3] if show_xy_axes else [3]
    for idx in axis_indices:
        cv2.arrowedLine(
            img, origin, tuple(proj_ax[idx]), AXIS_COLORS[idx - 1], 2, tipLength=0.25
        )

    ts = np.linspace(0, 2 * np.pi, 64, endpoint=False)
    circle_obj = np.stack(
        [radius_m * np.cos(ts), radius_m * np.sin(ts), np.zeros_like(ts)],
        axis=1,
        dtype=np.float64,
    )
    proj_c, _ = cv2.projectPoints(circle_obj, rvec, t, K, dist)
    cv2.polylines(
        img, [proj_c.reshape(-1, 2).astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA
    )


def draw_target(
    img: np.ndarray,
    target: TargetPose,
    inst_id: int,
    algorithm_id: int,
    K: np.ndarray,
    dist: np.ndarray,
    show_xy_axes: bool = False,
) -> None:
    radius_m = target.radius_m if target.radius_m > 0 else 0.025
    color = _mask_color(target.class_id)
    if target.mask is not None:
        draw_mask_overlay(img, target.mask, color)

    pos = target.pose_4x4[:3, 3]
    t_xyz, rpy_deg = pose_to_xyz_rpy_deg(target.pose_4x4)
    text_color = (0, 255, 0) if target.success else (0, 0, 255)
    u = v = None
    if target.success and pos[2] > 1e-6:
        u = int(pos[0] * K[0, 0] / pos[2] + K[0, 2])
        v = int(pos[1] * K[1, 1] / pos[2] + K[1, 2])
        cv2.circle(img, (u, v), 7, (0, 255, 0), 2, cv2.LINE_AA)
        _put_text_lines(
            img,
            [
                f"x={t_xyz[0]:.4f} y={t_xyz[1]:.4f} z={t_xyz[2]:.4f} m",
                f"roll={rpy_deg[0]:.1f} pitch={rpy_deg[1]:.1f} yaw={rpy_deg[2]:.1f} deg",
            ],
            (u + 10, v + 12),
            text_color,
            scale=0.45,
        )

    if target.algorithm_id == ALGO_PNP:
        if target.ellipse is not None:
            draw_ellipse_fit(img, target.ellipse)
        if target.success:
            draw_pnp_pose(img, target.pose_4x4, K, dist, radius_m, show_xy_axes=show_xy_axes)
    elif target.success and pos[2] > 1e-6:
        u = int(pos[0] * K[0, 0] / pos[2] + K[0, 2])
        v = int(pos[1] * K[1, 1] / pos[2] + K[1, 2])
        cv2.arrowedLine(img, (u, v), (u, v - 40), (255, 0, 0), 2, tipLength=0.3)

    status = "OK" if target.success else f"FAIL"
    label = target.class_name or f"cls{target.class_id}"
    y0 = 48 + inst_id * 56
    side_lines = [
        f"id{inst_id} {label} algo={target.algorithm_id} r={radius_m:.4f}m {status}",
    ]
    if target.success:
        side_lines.extend(
            [
                f"  pos(m): x={t_xyz[0]:.4f} y={t_xyz[1]:.4f} z={t_xyz[2]:.4f}",
                f"  rpy(deg): roll={rpy_deg[0]:.1f} pitch={rpy_deg[1]:.1f} yaw={rpy_deg[2]:.1f}",
            ]
        )
    else:
        side_lines.append(f"  {target.message}")
    _put_text_lines(img, side_lines, (10, y0), text_color, scale=0.48, line_h=17)


def draw_results(
    rgb: np.ndarray,
    output: AlgorithmOutput,
    K: np.ndarray,
    dist_coeffs: np.ndarray,
    show_xy_axes: bool = False,
) -> np.ndarray:
    vis = rgb.copy()
    K = np.asarray(K, dtype=np.float64)
    dist = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)

    for i, t in enumerate(output.targets):
        draw_target(vis, t, i, t.algorithm_id, K, dist, show_xy_axes=show_xy_axes)

    algos = sorted({t.algorithm_id for t in output.targets})
    cv2.putText(
        vis,
        f"algo={algos} n={len(output.targets)} | pos(m) rpy(deg) ZYX",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return vis
