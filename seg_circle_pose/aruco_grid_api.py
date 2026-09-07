"""
ArUco 网格 48 点 4×4 位姿 API（供 C++ pybind11 调用）。

输入：BGR 图像 + 相机内参 K/dist（单位：米）
输出：相机系下 48 个 4×4 齐次矩阵（row-major，单位：米）
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from aruco_realsense_online import (
    BOARD_AXIS_LENGTH_M,
    BOARD_MARKER_LAYOUT,
    GRID_X_STEP_COUNT,
    GRID_Y_STEP_COUNT,
    MARKER_DICTIONARY,
    MARKER_SIDE_LENGTH_M,
    SHOW_BOARD_GRID_POINTS,
    TARGET_IDS,
    USE_BOARD_POSE,
    BoardPose,
    GridModel,
    build_board_marker_centers,
    build_grid_model,
    draw_board_pose_overlay,
    draw_detected_markers,
    estimate_board_pose,
    filter_detections,
    get_aruco_dictionary,
    make_detector,
)

_detector = None
_dictionary = None
_marker_centers: dict[int, np.ndarray] | None = None
_grid_model: GridModel | None = None


def _ensure_models() -> None:
    global _detector, _dictionary, _marker_centers, _grid_model
    if _detector is None:
        _dictionary = get_aruco_dictionary(MARKER_DICTIONARY)
        _detector = make_detector(_dictionary)
        _marker_centers = build_board_marker_centers()
        _grid_model = build_grid_model()


def compute_grid_poses_4x4_cam(board_pose: BoardPose, grid_model: GridModel) -> np.ndarray:
    """48 个网格点在相机系下的 4×4 位姿（Z 轴与网格/ArUco 板法向一致）。"""
    R_a, _ = cv2.Rodrigues(board_pose.rvec.reshape(3, 1))
    t_a = board_pose.tvec.reshape(3)
    R_orient = R_a @ grid_model.R_grid_to_aruco
    points_aruco = grid_model.points_in_aruco_frame()

    n = len(points_aruco)
    out = np.zeros((n, 4, 4), dtype=np.float64)
    for i, p in enumerate(points_aruco):
        out[i, :3, :3] = R_orient
        out[i, :3, 3] = R_a @ p + t_a
        out[i, 3, 3] = 1.0
    return out


def detect_grid_poses_4x4_cam(
    bgr: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
) -> dict[str, Any]:
    """
    检测 ArUco Board 并返回 48 个网格点相机系 4×4 位姿。

    Parameters
    ----------
    bgr : (H,W,3) uint8 BGR
    K : (3,3) float64 内参，单位与标定一致（像素）
    dist : (5,) float64 畸变系数

    Returns
    -------
    dict
        ok: bool
        message: str
        num_grid: int
        marker_ids: list[int]
        num_corners: int
        reproj_px: float
        poses_4x4: (N,4,4) float64 相机系，米；失败时 shape (0,4,4)
    """
    _ensure_models()
    assert _detector is not None
    assert _marker_centers is not None
    assert _grid_model is not None

    if bgr is None or bgr.size == 0:
        return _fail("空图像")
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        return _fail(f"需要 BGR 三通道，当前 shape={getattr(bgr, 'shape', None)}")

    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    dist = np.asarray(dist, dtype=np.float64).reshape(-1)
    if dist.size < 5:
        dist = np.pad(dist, (0, 5 - dist.size))

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _detector.detectMarkers(gray)
    corners, ids = filter_detections(corners, ids, TARGET_IDS)

    if not USE_BOARD_POSE:
        return _fail("USE_BOARD_POSE=False 时暂不支持网格 API")

    board_pose = estimate_board_pose(
        corners, ids, _marker_centers, MARKER_SIDE_LENGTH_M, K, dist[:5],
    )
    if board_pose is None:
        return _fail("Board PnP 失败（码数/角点不足或未检测到目标码）")

    poses = compute_grid_poses_4x4_cam(board_pose, _grid_model)
    expected = GRID_X_STEP_COUNT * GRID_Y_STEP_COUNT
    if poses.shape[0] != expected:
        return _fail(f"网格点数异常: {poses.shape[0]} != {expected}")

    return {
        "ok": True,
        "message": "ok",
        "num_grid": int(poses.shape[0]),
        "marker_ids": [int(x) for x in board_pose.marker_ids],
        "num_corners": int(board_pose.num_corners),
        "reproj_px": float(board_pose.reproj_px),
        "poses_4x4": poses,
    }


def _fail(message: str) -> dict[str, Any]:
    return {
        "ok": False,
        "message": message,
        "num_grid": 0,
        "marker_ids": [],
        "num_corners": 0,
        "reproj_px": 0.0,
        "poses_4x4": np.zeros((0, 4, 4), dtype=np.float64),
    }


def draw_grid_detect_vis(
    bgr: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    trial_index: int = 0,
    detect_ok: bool = False,
    detect_message: str = "",
) -> np.ndarray:
    """放货 ArUco 调试：在原图上绘制检测框/板姿态/网格，供 C++ 第 4 调试窗格显示。"""
    if bgr is None or bgr.size == 0:
        vis = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(
            vis, "empty frame", (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA,
        )
        return vis

    vis = bgr.copy()
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    dist = np.asarray(dist, dtype=np.float64).reshape(-1)
    if dist.size < 5:
        dist = np.pad(dist, (0, 5 - dist.size))

    _ensure_models()
    assert _detector is not None
    assert _marker_centers is not None
    assert _grid_model is not None

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _detector.detectMarkers(gray)
    corners, ids = filter_detections(corners, ids, TARGET_IDS)
    draw_detected_markers(vis, corners, ids)

    board_pose = None
    if USE_BOARD_POSE:
        board_pose = estimate_board_pose(
            corners, ids, _marker_centers, MARKER_SIDE_LENGTH_M, K, dist[:5],
        )

    grid_model = _grid_model if SHOW_BOARD_GRID_POINTS else None
    vis = draw_board_pose_overlay(
        vis, board_pose, K, dist[:5], BOARD_AXIS_LENGTH_M, grid_model,
    )

    status = f"Place trial #{trial_index}"
    if detect_ok:
        status += " OK"
    else:
        status += f" FAIL: {detect_message or 'detect failed'}"
    cv2.putText(
        vis, status, (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA,
    )
    return vis
