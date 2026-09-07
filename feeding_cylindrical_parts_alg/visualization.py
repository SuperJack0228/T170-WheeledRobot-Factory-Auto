"""检测结果可视化：mask、PnP 椭圆/位姿轴、重心位置。"""

from __future__ import annotations

import cv2
import numpy as np

from algorithm.pose_pnp import EllipseFit
from algorithm.types import ALGO_PNP, AlgorithmOutput, TargetPose

AXIS_COLORS = ((0, 0, 255), (0, 255, 0), (255, 0, 0))  # X Y Z (BGR)
DRAW_LINE_THICKNESS = 1
MASK_COLORS = (
    (255, 128, 0),
    (0, 200, 255),
    (200, 0, 200),
    (0, 255, 128),
    (128, 128, 255),
)

CLASS_CN = {
    "original_product": "毛胚",
    "semi_finished_product": "半加工",
    "finished_product": "加工",
    "feeding_hole": "料盘孔",
}


def _mask_color(class_id: int) -> tuple[int, int, int]:
    return MASK_COLORS[class_id % len(MASK_COLORS)]


def _center_marker_color(color: tuple[int, int, int], scale: float = 0.38) -> tuple[int, int, int]:
    """圆心标记用更深色，便于与 mask/轮廓区分。"""
    return tuple(int(np.clip(c * scale, 0, 255)) for c in color)


def _draw_point_2x2(
    img: np.ndarray,
    u: int,
    v: int,
    color: tuple[int, int, int],
) -> None:
    """2×2 像素块（拟合圆心等）。"""
    h, w = img.shape[:2]
    u0, u1 = max(0, u), min(w, u + 2)
    v0, v1 = max(0, v), min(h, v + 2)
    if u0 < u1 and v0 < v1:
        img[v0:v1, u0:u1] = color


def _draw_center_point(
    img: np.ndarray,
    u: int,
    v: int,
    color: tuple[int, int, int],
) -> None:
    """位姿投影中心：单像素点。"""
    h, w = img.shape[:2]
    if 0 <= v < h and 0 <= u < w:
        img[v, u] = color


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
        cv2.drawContours(img, cnts, -1, color, DRAW_LINE_THICKNESS, cv2.LINE_AA)


def _project_pose_center_uv(
    pose_4x4: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
) -> tuple[float, float] | None:
    """物体原点 (0,0,0) 经 PnP 位姿重投影到图像 (u,v)。"""
    t = pose_4x4[:3, 3].astype(np.float64)
    if float(t[2]) <= 1e-6:
        return None
    rvec, _ = cv2.Rodrigues(pose_4x4[:3, :3].astype(np.float64))
    uv, _ = cv2.projectPoints(
        np.zeros((1, 3), dtype=np.float64),
        rvec,
        t.reshape(3, 1),
        K,
        dist,
    )
    pt = uv.reshape(2)
    return float(pt[0]), float(pt[1])


def _draw_pnp_center_comparison(
    img: np.ndarray,
    ell: EllipseFit,
    pose_4x4: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    class_color: tuple[int, int, int],
    text_color: tuple[int, int, int],
) -> None:
    """2D 拟合圆心 vs 3D 位姿圆心重投影，并标注像素差。"""
    uv3d = _project_pose_center_uv(pose_4x4, K, dist)
    if uv3d is None:
        return

    u2d, v2d = float(ell.center[0]), float(ell.center[1])
    u3d, v3d = uv3d
    du = u3d - u2d
    dv = v3d - v2d
    d_px = float(np.hypot(du, dv))

    # 2D 拟合圆心：深色 2×2；3D 重投影圆心：原类别色 2×2
    _draw_point_2x2(img, int(round(u2d)), int(round(v2d)), _center_marker_color(class_color))
    _draw_point_2x2(img, int(round(u3d)), int(round(v3d)), class_color)

    anchor_u = int(round((u2d + u3d) * 0.5))
    anchor_v = int(round((v2d + v3d) * 0.5))
    h = img.shape[0]
    _put_text_lines(
        img,
        [
            f"ctr_d={d_px:.2f}px",
            f"du={du:+.1f} dv={dv:+.1f}",
        ],
        (anchor_u + 8, max(14, min(h - 4, anchor_v - 4))),
        text_color,
        scale=0.42,
        line_h=14,
    )


def draw_ellipse_fit(
    img: np.ndarray,
    ell: EllipseFit,
    color: tuple[int, int, int],
    *,
    draw_center: bool = True,
) -> None:
    box = cv2.boxPoints(
        ((ell.center[0], ell.center[1]), ell.axes, ell.angle_deg)
    ).astype(np.int32)
    cv2.polylines(img, [box], True, color, DRAW_LINE_THICKNESS, cv2.LINE_AA)
    if draw_center:
        cx, cy = ell.center
        _draw_point_2x2(img, int(round(cx)), int(round(cy)), _center_marker_color(color))


def draw_pnp_pose(
    img: np.ndarray,
    pose_4x4: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    radius_m: float,
    color: tuple[int, int, int],
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
            img,
            origin,
            tuple(proj_ax[idx]),
            AXIS_COLORS[idx - 1],
            DRAW_LINE_THICKNESS,
            tipLength=0.25,
        )

    ts = np.linspace(0, 2 * np.pi, 64, endpoint=False)
    circle_obj = np.stack(
        [radius_m * np.cos(ts), radius_m * np.sin(ts), np.zeros_like(ts)],
        axis=1,
        dtype=np.float64,
    )
    proj_c, _ = cv2.projectPoints(circle_obj, rvec, t, K, dist)
    cv2.polylines(
        img,
        [proj_c.reshape(-1, 2).astype(np.int32)],
        True,
        color,
        DRAW_LINE_THICKNESS,
        cv2.LINE_AA,
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
    class_color = _mask_color(target.class_id)
    center_color = _center_marker_color(class_color)
    if target.mask is not None:
        draw_mask_overlay(img, target.mask, class_color)

    pos = target.pose_4x4[:3, 3]
    t_xyz, rpy_deg = pose_to_xyz_rpy_deg(target.pose_4x4)
    text_color = (0, 255, 0) if target.success else (0, 0, 255)

    if target.algorithm_id == ALGO_PNP:
        if target.ellipse is not None:
            draw_ellipse_fit(
                img,
                target.ellipse,
                class_color,
                draw_center=not target.success,
            )
        if target.success:
            draw_pnp_pose(
                img, target.pose_4x4, K, dist, radius_m, class_color, show_xy_axes=show_xy_axes
            )
            if target.ellipse is not None:
                _draw_pnp_center_comparison(
                    img,
                    target.ellipse,
                    target.pose_4x4,
                    K,
                    dist,
                    class_color,
                    text_color,
                )
    elif target.success and pos[2] > 1e-6:
        u = int(pos[0] * K[0, 0] / pos[2] + K[0, 2])
        v = int(pos[1] * K[1, 1] / pos[2] + K[1, 2])
        _draw_center_point(img, u, v, center_color)
        cv2.arrowedLine(
            img, (u, v), (u, v - 40), class_color, DRAW_LINE_THICKNESS, tipLength=0.3
        )

    status = "OK" if target.success else f"FAIL"
    raw = target.class_name or f"cls{target.class_id}"
    label = CLASS_CN.get(raw, raw)
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
