#!/usr/bin/env python3
"""
RealSense 在线 ArUco 检测 + 6D 位姿估计。

支持两种模式（USE_BOARD_POSE）:
  - True : 八码合成 Board，最多 32 角点一次 PnP（推荐）
  - False: 每个码单独 PnP

码图: https://chev.me/arucogen/  （当前为 Original ArUco 5x5 1000）

用法:
  cd /home/wt/my_project/factory/data/aruco/code
  python aruco_realsense_online.py              # 交互选择相机
  python aruco_realsense_online.py --list-cameras
  python aruco_realsense_online.py --camera-index 1
  python aruco_realsense_online.py --serial <序列号>

按键: q/ESC 退出 | s 保存截图 | b 切换 board/单码模式
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None  # C++ 传图模式不需要；仅在线 RealSense 脚本需要


def _require_realsense() -> None:
    if rs is None:
        raise RuntimeError("需要 pyrealsense2: pip install pyrealsense2")


# ========================= CONFIG（按需修改）=========================
# --- 单码 ---
MARKER_SIDE_LENGTH_M = 0.04          # 每个码边长（米），40mm → 0.04
MARKER_DICTIONARY = "DICT_5X5_1000"  # 与 arucogen 字典一致

# --- 八码 Board（内四码 + 外四码）---
# 像素平面（y 向下）上，面向屏幕逆时针绕行: 0 → 2 → 20 → 25
# 即: 左下(0) → 右下(2) → 右上(20) → 左上(25)
# 内四码（20cm 间距）示意:
#     ID=25 -------- ID=20
#      |                |
#     ID=0  --------  ID=2
# 外四码：以内四码中心为基准，沿 Board ±X、±Y 各再外扩 10cm（对角向外），
# 同样从 ID=0 起逆时针: 26 → 59 → 61 → 64
#     ID=64 -------- ID=61
#      |   ID=25  ID=20  |
#      |    |        |    |
#      |   ID=0   ID=2   |
#      |                |
#     ID=26 -------- ID=59
# Board 目标坐标系（右手系，原点=内四码中心）:
#   +X : 左下(ID=0) → 左上(ID=25)   （沿图像左列向上）
#   +Y : 右下(ID=2) → 左下(ID=0)   （沿图像下行向左）
#   +Z : 垂直码面向外（不变）
USE_BOARD_POSE = True

# 内四码相邻 **中心到中心** 间距（米）；当前横、纵均为 20cm
BOARD_GRID_SPACING_X_M = 0.20   # 图像下行: 左下 → 右下（沿 ±Y 方向）
BOARD_GRID_SPACING_Y_M = 0.20   # 图像左列: 左下 → 左上（沿 ±X 方向）

# 外四码相对内四码角点，沿 ±X、±Y 各外扩（米）
BOARD_OUTER_EXTENSION_M = 0.10


def _build_board_marker_layout() -> dict[int, tuple[str, float, float]]:
    """内四码 + 外四码在 Board 系下的中心坐标（原点=内四码几何中心）。"""
    hx = BOARD_GRID_SPACING_X_M / 2
    hy = BOARD_GRID_SPACING_Y_M / 2
    ext = BOARD_OUTER_EXTENSION_M
    return {
        0: ("inner_bottom_left", -hy, +hx),
        2: ("inner_bottom_right", -hy, -hx),
        20: ("inner_top_right", +hy, -hx),
        25: ("inner_top_left", +hy, +hx),
        # 从 ID=0 起逆时针，沿 ±X、±Y 各外扩 ext
        26: ("outer_bottom_left", -hy - ext, +hx + ext),
        59: ("outer_bottom_right", -hy - ext, -hx - ext),
        61: ("outer_top_right", +hy + ext, -hx - ext),
        64: ("outer_top_left", +hy + ext, +hx + ext),
    }


BOARD_MARKER_LAYOUT: dict[int, tuple[str, float, float]] = _build_board_marker_layout()

BOARD_MIN_MARKERS = 2       # 至少检测到几个 board 码才解算
BOARD_MIN_CORNERS = 8       # 至少多少个角点（2 个码）
BOARD_PNP_FLAG = "SQPNP"    # SQPNP | ITERATIVE | IPPE_SQUARE（仅 4 点时用）
BOARD_PNP_REFINE = True     # 初解后用 LM 迭代精化（角点多时更稳）

# board 坐标轴可视化长度（米）
BOARD_AXIS_LENGTH_M = 0.8
# 单码坐标轴 = 边长 × 比例
SINGLE_AXIS_LENGTH_RATIO = 0.5

# --- 网格坐标系（与 ArUco PnP 坐标系分离，Z=0 平面）---
# 原点：四码区域最右下角；+X 前；+Y 左；+Z 朝外（右手系）
SHOW_BOARD_GRID_POINTS = True
SHOW_GRID_FRAME_AXES = True
GRID_POINT_SPACING_M = 0.10       # 点间距 10cm
GRID_X_STEP_COUNT = 6             # 沿 +X（前）: 0,10,...,50cm
GRID_Y_STEP_COUNT = 8             # 沿 +Y（左）: 0,10,...,70cm → 6×8=48 点
GRID_DRAW_LINES = True
GRID_POINT_RADIUS = 2
GRID_POINT_COLOR = (255, 0, 255)   # BGR 洋红
GRID_LINE_COLOR = (200, 100, 255)
GRID_AXIS_LENGTH_M = 0.08

# ArUco Board 原点（四码中心）在「网格坐标系」下的位置（米）。
# 从网格起点（右下角）看：向前 +X、向左 +Y 到达 ArUco 原点。
# 设为 None 则按 BOARD 几何自动估算；手调时只改下面三个数即可。
ARUCO_ORIGIN_IN_GRID_X_M: float | None = 0.250
ARUCO_ORIGIN_IN_GRID_Y_M: float | None = 0.350
ARUCO_ORIGIN_IN_GRID_Z_M: float | None = 0.0
# 网格轴相对 ArUco Board 轴绕 Z 的额外旋转（度），轴向一致时保持 0
GRID_TO_ARUCO_YAW_DEG = 0.0
# 在自动估算的 ArUco 原点位置上再叠加微调（米，网格系）
ARUCO_ORIGIN_IN_GRID_FINE_X_M = 0.0
ARUCO_ORIGIN_IN_GRID_FINE_Y_M = 0.0
ARUCO_ORIGIN_IN_GRID_FINE_Z_M = 0.0

# 检测时只保留这些 ID；None = 全部
TARGET_IDS: set[int] | None = {0, 2, 20, 25, 26, 59, 61, 64}

# --- 相机 ---
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30
# 指定序列号则跳过交互选择；留空 "" 则启动时列出所有 RealSense 供选择
CAMERA_SERIAL = ""

OUTPUT_DIR = Path(__file__).resolve().parent / "captures"
# ===================================================================


PNP_FLAGS = {
    "SQPNP": cv2.SOLVEPNP_SQPNP,
    "ITERATIVE": cv2.SOLVEPNP_ITERATIVE,
    "IPPE_SQUARE": cv2.SOLVEPNP_IPPE_SQUARE,
}


@dataclass
class RealSenseDevice:
    serial: str
    name: str

    @property
    def label(self) -> str:
        short = self.serial if len(self.serial) <= 14 else f"{self.serial[:10]}..."
        return f"{self.name} [{short}]"


@dataclass
class MarkerPose:
    marker_id: int
    rvec: np.ndarray
    tvec: np.ndarray
    corners: np.ndarray


@dataclass
class BoardPose:
    rvec: np.ndarray
    tvec: np.ndarray
    marker_ids: list[int]
    num_corners: int
    reproj_px: float


@dataclass
class GridModel:
    """网格定义 + ArUco 原点在网格系下的偏置，用于投影到相机。"""

    points_grid: np.ndarray
    x_coords: np.ndarray
    y_coords: np.ndarray
    aruco_origin_in_grid: np.ndarray
    R_grid_to_aruco: np.ndarray

    def points_in_aruco_frame(self) -> np.ndarray:
        delta = self.points_grid - self.aruco_origin_in_grid.reshape(1, 3)
        return (self.R_grid_to_aruco @ delta.T).T.astype(np.float32)

    def grid_origin_in_aruco_frame(self) -> np.ndarray:
        return (-self.R_grid_to_aruco @ self.aruco_origin_in_grid.reshape(3, 1)).reshape(3)


def get_aruco_dictionary(name: str) -> cv2.aruco.Dictionary:
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"未知 ArUco 字典: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_detector(dictionary: cv2.aruco.Dictionary) -> cv2.aruco.ArucoDetector:
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary, params)


def rotation_matrix_to_euler_zyx_deg(R: np.ndarray) -> tuple[float, float, float]:
    sy = float(np.hypot(R[0, 0], R[1, 0]))
    if sy > 1e-6:
        yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
        roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
    else:
        yaw = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
        roll = 0.0
    return yaw, pitch, roll


def marker_corners_local(marker_length_m: float) -> np.ndarray:
    """单码四角在码中心坐标系下的 3D 坐标，与 OpenCV 角点顺序一致。"""
    h = marker_length_m * 0.5
    return np.array(
        [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
        dtype=np.float32,
    )


def build_board_marker_centers() -> dict[int, np.ndarray]:
    centers: dict[int, np.ndarray] = {}
    for mid, (_name, cx, cy) in BOARD_MARKER_LAYOUT.items():
        centers[mid] = np.array([cx, cy, 0.0], dtype=np.float32)
    return centers


def rotation_z_rad(yaw_rad: float) -> np.ndarray:
    c, s = float(np.cos(yaw_rad)), float(np.sin(yaw_rad))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def estimate_aruco_origin_in_grid() -> np.ndarray:
    """
    自动估算：ArUco Board 原点（四码中心）在网格系下的坐标。
    网格起点 = ID=2 右下码中心再沿 ArUco -X/-Y 各半个码边（最外右下角）。
    网格 +X/+Y 与 ArUco +X/+Y 同向时，结果为 (sy/2+h, sx/2+h, 0)。
    """
    h = MARKER_SIDE_LENGTH_M * 0.5
    auto_x = BOARD_GRID_SPACING_Y_M / 2 + h
    auto_y = BOARD_GRID_SPACING_X_M / 2 + h
    auto_z = 0.0
    x = auto_x if ARUCO_ORIGIN_IN_GRID_X_M is None else ARUCO_ORIGIN_IN_GRID_X_M
    y = auto_y if ARUCO_ORIGIN_IN_GRID_Y_M is None else ARUCO_ORIGIN_IN_GRID_Y_M
    z = auto_z if ARUCO_ORIGIN_IN_GRID_Z_M is None else ARUCO_ORIGIN_IN_GRID_Z_M
    return np.array(
        [
            x + ARUCO_ORIGIN_IN_GRID_FINE_X_M,
            y + ARUCO_ORIGIN_IN_GRID_FINE_Y_M,
            z + ARUCO_ORIGIN_IN_GRID_FINE_Z_M,
        ],
        dtype=np.float64,
    )


def build_grid_model() -> GridModel:
    """网格点先在网格系生成，再通过 ArUco 偏置变换到 ArUco 系后投影。"""
    s = GRID_POINT_SPACING_M
    x_coords = (np.arange(GRID_X_STEP_COUNT, dtype=np.float32) * s)
    y_coords = (np.arange(GRID_Y_STEP_COUNT, dtype=np.float32) * s)
    xx, yy = np.meshgrid(x_coords, y_coords, indexing="xy")
    zz = np.zeros_like(xx)
    points = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1).astype(np.float32)
    R = rotation_z_rad(np.deg2rad(GRID_TO_ARUCO_YAW_DEG))
    origin = estimate_aruco_origin_in_grid()
    return GridModel(
        points_grid=points,
        x_coords=x_coords,
        y_coords=y_coords,
        aruco_origin_in_grid=origin,
        R_grid_to_aruco=R,
    )


def grid_origin_pose_in_camera(
    aruco_rvec: np.ndarray,
    aruco_tvec: np.ndarray,
    grid_model: GridModel,
) -> tuple[np.ndarray, np.ndarray]:
    """由 ArUco 位姿 + 网格偏置，得到网格原点在相机系下的 rvec/tvec。"""
    R_a, _ = cv2.Rodrigues(aruco_rvec)
    t_go = grid_model.grid_origin_in_aruco_frame().reshape(3, 1)
    t_grid_cam = (R_a @ t_go + aruco_tvec.reshape(3, 1)).reshape(3, 1)
    if abs(GRID_TO_ARUCO_YAW_DEG) < 1e-6:
        rvec_grid = aruco_rvec.reshape(3, 1)
    else:
        R_g = grid_model.R_grid_to_aruco
        R_grid_cam = R_a @ R_g
        rvec_grid, _ = cv2.Rodrigues(R_grid_cam)
    return rvec_grid.reshape(3), t_grid_cam.reshape(3)


def project_board_points(
    points_3d: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Board 系 3D 点 → 图像像素；返回 (N,2), valid(N,) 在图像范围内且 Z_cam>0。"""
    if len(points_3d) == 0:
        return np.zeros((0, 2), dtype=np.float32), np.zeros(0, dtype=bool)

    tvec_col = tvec.reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    pts_cam = (R @ points_3d.T + tvec_col).T
    valid_z = pts_cam[:, 2] > 1e-4

    img_pts, _ = cv2.projectPoints(points_3d, rvec, tvec_col, K, dist)
    img_pts = img_pts.reshape(-1, 2)
    return img_pts, valid_z


def draw_board_grid_on_image(
    vis: np.ndarray,
    grid_model: GridModel,
    aruco_rvec: np.ndarray,
    aruco_tvec: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
) -> None:
    points_aruco = grid_model.points_in_aruco_frame()
    x_coords, y_coords = grid_model.x_coords, grid_model.y_coords
    img_pts, valid_z = project_board_points(
        points_aruco, aruco_rvec, aruco_tvec, K, dist,
    )
    h, w = vis.shape[:2]
    in_frame = (
        valid_z
        & (img_pts[:, 0] >= 0) & (img_pts[:, 0] < w)
        & (img_pts[:, 1] >= 0) & (img_pts[:, 1] < h)
    )

    if GRID_DRAW_LINES and len(x_coords) > 1 and len(y_coords) > 1:
        nx, ny = len(x_coords), len(y_coords)
        pts_grid = img_pts.reshape(ny, nx, 2)

        def _draw_seg(i0: int, j0: int, i1: int, j1: int) -> None:
            idx0, idx1 = j0 * nx + i0, j1 * nx + i1
            if not (in_frame[idx0] and in_frame[idx1]):
                return
            p0 = tuple(np.round(pts_grid[j0, i0]).astype(int))
            p1 = tuple(np.round(pts_grid[j1, i1]).astype(int))
            cv2.line(vis, p0, p1, GRID_LINE_COLOR, 1, cv2.LINE_AA)

        for j in range(ny):
            for i in range(nx - 1):
                _draw_seg(i, j, i + 1, j)
        for i in range(nx):
            for j in range(ny - 1):
                _draw_seg(i, j, i, j + 1)

    for k, (u, v) in enumerate(img_pts):
        if not in_frame[k]:
            continue
        center = (int(round(u)), int(round(v)))
        cv2.circle(vis, center, GRID_POINT_RADIUS, GRID_POINT_COLOR, -1, cv2.LINE_AA)
        cv2.circle(vis, center, GRID_POINT_RADIUS + 1, (255, 255, 255), 1, cv2.LINE_AA)


def mean_reprojection_error(
    obj_pts: np.ndarray,
    img_pts: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
) -> float:
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    err = np.linalg.norm(proj - img_pts.reshape(-1, 2), axis=1)
    return float(np.mean(err))


def collect_board_correspondences(
    corners,
    ids,
    marker_centers: dict[int, np.ndarray],
    marker_length_m: float,
) -> tuple[np.ndarray, np.ndarray, list[int]] | None:
    if ids is None or len(corners) == 0:
        return None

    local = marker_corners_local(marker_length_m)
    obj_list: list[np.ndarray] = []
    img_list: list[np.ndarray] = []
    used_ids: list[int] = []

    for i, mid_raw in enumerate(ids.flatten()):
        mid = int(mid_raw)
        if mid not in marker_centers:
            continue
        obj_list.append(local + marker_centers[mid])
        img_list.append(corners[i].reshape(4, 2).astype(np.float32))
        used_ids.append(mid)

    if not used_ids:
        return None

    obj_pts = np.vstack(obj_list).astype(np.float32)
    img_pts = np.vstack(img_list).astype(np.float32)
    return obj_pts, img_pts, used_ids


def estimate_board_pose(
    corners,
    ids,
    marker_centers: dict[int, np.ndarray],
    marker_length_m: float,
    K: np.ndarray,
    dist: np.ndarray,
) -> BoardPose | None:
    corr = collect_board_correspondences(corners, ids, marker_centers, marker_length_m)
    if corr is None:
        return None
    obj_pts, img_pts, used_ids = corr

    if len(used_ids) < BOARD_MIN_MARKERS or len(obj_pts) < BOARD_MIN_CORNERS:
        return None

    flag = PNP_FLAGS.get(BOARD_PNP_FLAG, cv2.SOLVEPNP_SQPNP)
    if len(obj_pts) == 4 and flag == cv2.SOLVEPNP_SQPNP:
        flag = cv2.SOLVEPNP_IPPE_SQUARE

    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist, flags=flag)
    if not ok:
        return None

    if BOARD_PNP_REFINE and hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(
            obj_pts, img_pts, K, dist, rvec, tvec,
        )

    reproj = mean_reprojection_error(obj_pts, img_pts, rvec, tvec, K, dist)
    return BoardPose(
        rvec=rvec.reshape(3),
        tvec=tvec.reshape(3),
        marker_ids=used_ids,
        num_corners=len(obj_pts),
        reproj_px=reproj,
    )


def estimate_single_marker_poses(
    corners,
    ids,
    marker_length_m: float,
    K: np.ndarray,
    dist: np.ndarray,
) -> list[MarkerPose]:
    if ids is None or len(corners) == 0:
        return []
    obj_pts = marker_corners_local(marker_length_m)
    out: list[MarkerPose] = []
    for i in range(len(ids)):
        img_pts = corners[i].reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            continue
        out.append(
            MarkerPose(
                marker_id=int(ids[i][0]),
                rvec=rvec.reshape(3),
                tvec=tvec.reshape(3),
                corners=corners[i],
            )
        )
    return out


def draw_detected_markers(vis: np.ndarray, corners, ids) -> None:
    if ids is not None and len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)


def draw_board_pose_overlay(
    image: np.ndarray,
    board_pose: BoardPose | None,
    K: np.ndarray,
    dist: np.ndarray,
    axis_len_m: float,
    grid_model: GridModel | None = None,
) -> np.ndarray:
    vis = image.copy()
    if board_pose is None:
        cv2.putText(
            vis, "Board pose: N/A", (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA,
        )
        return vis

    rvec = board_pose.rvec
    tvec = board_pose.tvec.reshape(3, 1)
    cv2.drawFrameAxes(vis, K, dist, rvec, tvec, axis_len_m)

    # board 原点投影
    origin_2d, _ = cv2.projectPoints(
        np.array([[0.0, 0.0, 0.0]], dtype=np.float32), rvec, tvec, K, dist,
    )
    ox, oy = int(origin_2d[0, 0, 0]), int(origin_2d[0, 0, 1])
    cv2.circle(vis, (ox, oy), 6, (0, 255, 255), -1)
    cv2.putText(vis, "BOARD", (ox + 8, oy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    R, _ = cv2.Rodrigues(rvec)
    yaw, pitch, roll = rotation_matrix_to_euler_zyx_deg(R)
    tx, ty, tz = (board_pose.tvec * 1000.0).tolist()
    dist_mm = float(np.linalg.norm(board_pose.tvec) * 1000.0)

    if SHOW_BOARD_GRID_POINTS and grid_model is not None:
        draw_board_grid_on_image(vis, grid_model, rvec, tvec, K, dist)
        if SHOW_GRID_FRAME_AXES:
            rvec_g, tvec_g = grid_origin_pose_in_camera(rvec, tvec, grid_model)
            cv2.drawFrameAxes(vis, K, dist, rvec_g, tvec_g.reshape(3, 1), GRID_AXIS_LENGTH_M)
            g2d, _ = cv2.projectPoints(
                np.zeros((1, 3), np.float32), rvec_g, tvec_g.reshape(3, 1), K, dist,
            )
            gx, gy = int(g2d[0, 0, 0]), int(g2d[0, 0, 1])
            cv2.circle(vis, (gx, gy), 6, (0, 165, 255), -1)
            cv2.putText(vis, "GRID0", (gx + 8, gy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    grid_info = ""
    origin_txt = ""
    if grid_model is not None:
        grid_info = f" | grid={len(grid_model.points_grid)}pts"
        og = grid_model.aruco_origin_in_grid * 1000.0
        origin_txt = f"aruco@grid(mm)=({og[0]:+.0f},{og[1]:+.0f},{og[2]:+.0f})"

    lines = [
        f"BOARD ids={board_pose.marker_ids} corners={board_pose.num_corners}{grid_info}",
        f"t(mm)=({tx:+.0f},{ty:+.0f},{tz:+.0f}) dist={dist_mm:.0f}mm",
        f"rpy(deg)=({yaw:+.1f},{pitch:+.1f},{roll:+.1f})",
        f"reproj={board_pose.reproj_px:.2f}px",
    ]
    if origin_txt:
        lines.append(origin_txt)
    for j, line in enumerate(lines):
        cv2.putText(
            vis, line, (10, 60 + j * 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA,
        )
    return vis


def draw_single_pose_overlay(
    image: np.ndarray,
    poses: list[MarkerPose],
    K: np.ndarray,
    dist: np.ndarray,
    axis_len_m: float,
) -> np.ndarray:
    vis = image.copy()
    for p in poses:
        cv2.drawFrameAxes(vis, K, dist, p.rvec, p.tvec.reshape(3, 1), axis_len_m)
        R, _ = cv2.Rodrigues(p.rvec)
        yaw, pitch, roll = rotation_matrix_to_euler_zyx_deg(R)
        tx, ty, tz = (p.tvec * 1000.0).tolist()
        dist_mm = float(np.linalg.norm(p.tvec) * 1000.0)

        c = p.corners.reshape(-1, 2).astype(int)
        x0 = int(np.clip(c[:, 0].min(), 0, vis.shape[1] - 1))
        y0 = int(np.clip(c[:, 1].min() - 8, 12, vis.shape[0] - 1))
        lines = [
            f"ID={p.marker_id}",
            f"t(mm)=({tx:+.0f},{ty:+.0f},{tz:+.0f})",
            f"dist={dist_mm:.0f}mm",
            f"rpy=({yaw:+.1f},{pitch:+.1f},{roll:+.1f})",
        ]
        for j, line in enumerate(lines):
            cv2.putText(
                vis, line, (x0, y0 + j * 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA,
            )
    return vis


def list_realsense_devices() -> list[RealSenseDevice]:
    _require_realsense()
    devices: list[RealSenseDevice] = []
    for dev in rs.context().query_devices():
        try:
            name = dev.get_info(rs.camera_info.name)
            serial = dev.get_info(rs.camera_info.serial_number)
            if "platform camera" in name.lower():
                continue
            devices.append(RealSenseDevice(serial=serial, name=name))
        except Exception:
            continue
    return devices


def select_realsense_device(
    serial: str = "",
    camera_index: int | None = None,
) -> RealSenseDevice:
    devices = list_realsense_devices()
    if not devices:
        raise RuntimeError("未检测到 RealSense 相机，请检查 USB 连接")

    if serial:
        for d in devices:
            if d.serial == serial:
                return d
        raise RuntimeError(f"未找到序列号为 {serial} 的相机，当前: {[d.serial for d in devices]}")

    if camera_index is not None:
        if camera_index < 0 or camera_index >= len(devices):
            raise RuntimeError(f"camera-index 越界: {camera_index}，共 {len(devices)} 台")
        return devices[camera_index]

    if len(devices) == 1:
        print(f"仅 1 台相机，自动选择: {devices[0].label}")
        return devices[0]

    print("检测到 RealSense 相机:")
    for i, d in enumerate(devices):
        print(f"  [{i}] {d.label}  serial={d.serial}")
    while True:
        raw = input(f"请选择相机 [0-{len(devices) - 1}]（默认 0）: ").strip()
        if raw == "":
            return devices[0]
        if raw.isdigit():
            idx = int(raw)
            if 0 <= idx < len(devices):
                return devices[idx]
        print("输入无效，请重新输入")


class RealSenseRgb:
    def __init__(
        self,
        width: int,
        height: int,
        fps: int,
        device: RealSenseDevice,
    ) -> None:
        _require_realsense()
        self.device = device
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(device.serial)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        profile = self._pipeline.start(cfg)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = np.array(
            [[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self.dist = np.array(list(intr.coeffs[:5]), dtype=np.float64)

    def read(self) -> np.ndarray | None:
        frames = self._pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            return None
        return np.asanyarray(color.get_data())

    def stop(self) -> None:
        self._pipeline.stop()


def filter_detections(corners, ids, allowed: set[int] | None):
    if ids is None or allowed is None:
        return corners, ids
    keep = [i for i, mid in enumerate(ids.flatten()) if int(mid) in allowed]
    if not keep:
        return [], None
    corners = [corners[i] for i in keep]
    ids = np.array([[int(ids[i][0])] for i in keep], dtype=np.int32)
    return corners, ids


def print_startup(use_board: bool, device: RealSenseDevice | None = None) -> None:
    print("ArUco RealSense 在线定位")
    if device is not None:
        print(f"  相机: {device.label}")
        print(f"  序列号: {device.serial}")
    print(f"  模式: {'八码 Board 融合' if use_board else '单码独立 PnP'}")
    print(f"  字典: {MARKER_DICTIONARY}")
    print(f"  码边长: {MARKER_SIDE_LENGTH_M * 1000:.1f} mm")
    if use_board:
        print(
            f"  Board 间距: 行={BOARD_GRID_SPACING_X_M*1000:.0f}mm(+Y) "
            f"列={BOARD_GRID_SPACING_Y_M*1000:.0f}mm(+X) "
            f"外扩={BOARD_OUTER_EXTENSION_M*1000:.0f}mm"
        )
        if BOARD_PNP_REFINE:
            print("  PnP 精化: solvePnPRefineLM 已启用")
        print("  Board 坐标: +X=左下→左上, +Y=右下→左下, +Z=朝外")
        if SHOW_BOARD_GRID_POINTS:
            gm = build_grid_model()
            og = gm.aruco_origin_in_grid * 1000.0
            print(
                f"  网格: 起点=右下, +X前 +Y左, 间距={GRID_POINT_SPACING_M*1000:.0f}mm "
                f"({GRID_X_STEP_COUNT}x{GRID_Y_STEP_COUNT}={len(gm.points_grid)}pts)"
            )
            print(
                f"  ArUco原点在网格系(mm): ({og[0]:+.1f}, {og[1]:+.1f}, {og[2]:+.1f}) "
                f"{'[自动]' if ARUCO_ORIGIN_IN_GRID_X_M is None else '[手设]'}"
            )
        print(f"  Board 布局: { {k: v[0] for k, v in BOARD_MARKER_LAYOUT.items()} }")
        print(f"  最少码数/角点: {BOARD_MIN_MARKERS} / {BOARD_MIN_CORNERS}")
    print(f"  跟踪 ID: {sorted(TARGET_IDS) if TARGET_IDS else '全部'}")
    print(f"  分辨率: {CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}")
    print("  q/ESC 退出 | s 保存截图 | b 切换 board/单码模式")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RealSense 在线 ArUco / Board 定位")
    p.add_argument(
        "--serial",
        type=str,
        default=CAMERA_SERIAL,
        help="相机序列号；不指定则交互选择",
    )
    p.add_argument(
        "--camera-index",
        type=int,
        default=None,
        help="相机列表下标（非交互，优先级低于 --serial）",
    )
    p.add_argument(
        "--list-cameras",
        action="store_true",
        help="列出所有 RealSense 后退出",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if args.list_cameras:
        devices = list_realsense_devices()
        if not devices:
            print("未检测到 RealSense 相机")
            return 1
        for i, d in enumerate(devices):
            print(f"[{i}] {d.label}  serial={d.serial}")
        return 0

    device = select_realsense_device(serial=args.serial, camera_index=args.camera_index)

    dictionary = get_aruco_dictionary(MARKER_DICTIONARY)
    detector = make_detector(dictionary)
    marker_centers = build_board_marker_centers()
    grid_model = build_grid_model()
    use_board = USE_BOARD_POSE
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    print_startup(use_board, device)

    cam = RealSenseRgb(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, device)
    win = "ArUco RealSense"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    try:
        while True:
            bgr = cam.read()
            if bgr is None:
                continue

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            corners, ids = filter_detections(corners, ids, TARGET_IDS)

            vis = bgr.copy()
            draw_detected_markers(vis, corners, ids)

            if use_board:
                board_pose = estimate_board_pose(
                    corners, ids, marker_centers, MARKER_SIDE_LENGTH_M, cam.K, cam.dist,
                )
                vis = draw_board_pose_overlay(
                    vis, board_pose, cam.K, cam.dist, BOARD_AXIS_LENGTH_M,
                    grid_model=grid_model,
                )
                mode = "BOARD"
                n_det = len(board_pose.marker_ids) if board_pose else 0
            else:
                poses = estimate_single_marker_poses(
                    corners, ids, MARKER_SIDE_LENGTH_M, cam.K, cam.dist,
                )
                axis_len = MARKER_SIDE_LENGTH_M * SINGLE_AXIS_LENGTH_RATIO
                vis = draw_single_pose_overlay(vis, poses, cam.K, cam.dist, axis_len)
                mode = "SINGLE"
                n_det = len(poses)

            short_serial = cam.device.serial[-6:]
            status = (
                f"{mode} | cam=...{short_serial} | det={n_det} | "
                f"L={MARKER_SIDE_LENGTH_M*1000:.0f}mm | "
                f"grid={BOARD_GRID_SPACING_X_M*1000:.0f}x{BOARD_GRID_SPACING_Y_M*1000:.0f}mm"
            )
            cv2.putText(
                vis, status, (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA,
            )

            cv2.imshow(win, vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                out = OUTPUT_DIR / f"aruco_{ts}.png"
                cv2.imwrite(str(out), vis)
                print(f"已保存: {out}")
            if key == ord("b"):
                use_board = not use_board
                print_startup(use_board, device)
    finally:
        cam.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
