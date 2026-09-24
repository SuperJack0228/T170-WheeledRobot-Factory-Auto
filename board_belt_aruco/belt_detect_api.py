"""Head-camera conveyor ArUco API (DICT_6X6). Independent of tray 5x5.

Loads board_belt_aruco/online.py under a unique module name so it never
shadows board_yf100_aruco's `online` (tray_detect_api).
"""

from __future__ import annotations

import importlib.util
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
_ONLINE_NAME = "t170c_board_belt_aruco_online"


def _load_online():
    existing = sys.modules.get(_ONLINE_NAME)
    if existing is not None:
        return existing
    path = HERE / "online.py"
    spec = importlib.util.spec_from_file_location(_ONLINE_NAME, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载传送带 online.py: {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[_ONLINE_NAME] = mod
    spec.loader.exec_module(mod)
    return mod


online = _load_online()

_cfg = None
_detector = None


def init(config_path: str | Path | None = None) -> str:
    global _cfg, _detector
    path = Path(config_path) if config_path else HERE / "config.yaml"
    _cfg = online.load_config(path)
    _detector = online.make_detector(online.get_aruco_dictionary(_cfg.dictionary))
    names = ",".join(b.name for b in _cfg.belts)
    return (
        f"dict={_cfg.dictionary} belts={names} "
        f"pitch_y={_cfg.pitch_y_m:.3f}m black={_cfg.black_side_m * 1000:.2f}mm "
        f"ids={sorted(_cfg.all_ids)}"
    )


def _require():
    if _cfg is None or _detector is None:
        raise RuntimeError("belt engine is not initialized")
    return _cfg, _detector


def _as_K(K: np.ndarray) -> np.ndarray:
    return np.asarray(K, dtype=np.float64).reshape(3, 3)


def _as_dist(dist: np.ndarray | None) -> np.ndarray:
    if dist is None:
        return np.zeros(5, dtype=np.float64)
    arr = np.asarray(dist, dtype=np.float64).reshape(-1)
    if arr.size < 5:
        arr = np.pad(arr, (0, 5 - arr.size))
    return arr[:5]


def _as_T(cam2robot: np.ndarray | None) -> np.ndarray | None:
    if cam2robot is None:
        return None
    return np.asarray(cam2robot, dtype=np.float64).reshape(4, 4)


def _xyz_robot(T_rc: np.ndarray | None, xyz_cam: np.ndarray) -> list[float] | None:
    if T_rc is None:
        return None
    p = np.array([xyz_cam[0], xyz_cam[1], xyz_cam[2], 1.0], dtype=np.float64)
    return (T_rc @ p)[:3].tolist()


def _R_robot(T_rc: np.ndarray | None, rvec: np.ndarray) -> list[float] | None:
    """识别板旋转到手臂基座（行主序 3×3）：列向量是板 +X前 / +Y左 / +Z上。"""
    if T_rc is None:
        return None
    R_cam, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    R = T_rc[:3, :3] @ R_cam
    return R.reshape(-1).tolist()


def default_save_path(project_root: str) -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    return str(Path(project_root) / "picture_debug" / "head" / f"belt_aruco_{ts}.jpg")


def _grasp_from_pose(pose, T_rc: np.ndarray | None, offset_board) -> tuple[np.ndarray, list[float] | None]:
    """抓取点 = 码原点 + R_识别 × offset。offset 在识别板系：+X 前 +Y 左 +Z 上。

    先转腰再拍照：原点/R 已在当前手臂基座，不再乘腰 Rz。
    """
    off = np.asarray(offset_board, dtype=np.float64).reshape(3)
    R_cam, _ = cv2.Rodrigues(np.asarray(pose.rvec, dtype=np.float64).reshape(3, 1))
    origin_cam = pose.tvec.reshape(3).astype(np.float64)
    grasp_cam = origin_cam + R_cam @ off
    return grasp_cam, _xyz_robot(T_rc, grasp_cam)


def _as_off3(offset) -> np.ndarray | None:
    if offset is None:
        return None
    off = np.asarray(offset, dtype=np.float64).reshape(-1)
    if off.size < 3 or not np.all(np.isfinite(off[:3])):
        return None
    return off[:3]


def _draw_grasp(vis, K, dist, grasp_cam: np.ndarray, label: str, color) -> None:
    pix, _ = cv2.projectPoints(
        grasp_cam.reshape(1, 3),
        np.zeros(3),
        np.zeros(3),
        K,
        dist,
    )
    u, v = int(round(float(pix[0, 0, 0]))), int(round(float(pix[0, 0, 1])))
    cv2.circle(vis, (u, v), 8, color, 2, cv2.LINE_AA)
    cv2.drawMarker(vis, (u, v), color, cv2.MARKER_TILTED_CROSS, 18, 2)
    cv2.putText(
        vis, label, (u + 10, v - 8),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA,
    )


def detect_frame(
    bgr,
    K,
    dist,
    cam2robot=None,
    save_path: str | None = None,
    prefer_name: str = "belt0",
    grasp_offset=None,
    grasp_offset_left=None,
    grasp_offset_right=None,
) -> dict[str, Any]:
    """Detect conveyor 6x6 markers. Does not use tray 5x5 or YOLO.

    grasp_offset[_left/_right]: 识别板系 (x前, y左, z上)，米。Y+ 向左横跨皮带。
    """
    cfg, detector = _require()
    bgr = np.ascontiguousarray(bgr)
    K = _as_K(K)
    dist = _as_dist(dist)
    T_rc = _as_T(cam2robot)

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    vis = bgr.copy()
    online.draw_detected_markers_thin(vis, corners, ids, cfg.all_ids)

    found: list[dict[str, Any]] = []
    for i, belt in enumerate(cfg.belts):
        pose = online.estimate_belt_pose(corners, ids, cfg, belt, K, dist)
        if pose is None:
            continue
        color = online.BELT_COLORS[i % len(online.BELT_COLORS)]
        online.overlay_belt(vis, pose, belt, cfg, K, dist, color, (8, 34 + i * 48))
        origin_cam = pose.tvec.reshape(3).astype(np.float64)
        found.append(
            {
                "name": pose.name,
                "used_ids": list(pose.used_ids),
                "reproj_px": float(pose.reproj_px),
                "origin_cam": origin_cam.tolist(),
                "origin_robot": _xyz_robot(T_rc, origin_cam),
                "R_robot": _R_robot(T_rc, pose.rvec),
                "rvec": pose.rvec.reshape(3).tolist(),
                "tvec": origin_cam.tolist(),
            }
        )

    chosen = None
    prefer = str(prefer_name or "").strip()
    if prefer:
        for item in found:
            if item["name"] == prefer:
                chosen = item
                break
    if chosen is None and found:
        chosen = found[0]

    grasp_cam_l = None
    grasp_robot_l = None
    grasp_cam_r = None
    grasp_robot_r = None
    off_l = _as_off3(grasp_offset_left)
    off_r = _as_off3(grasp_offset_right)
    off_one = _as_off3(grasp_offset)
    if off_l is None:
        off_l = off_one
    if off_r is None:
        off_r = off_one
    if chosen is not None and (off_l is not None or off_r is not None):
        class _P:
            pass
        pose = _P()
        pose.rvec = np.asarray(chosen["rvec"], dtype=np.float64)
        pose.tvec = np.asarray(chosen["tvec"], dtype=np.float64)
        if off_l is not None:
            grasp_cam_l, grasp_robot_l = _grasp_from_pose(pose, T_rc, off_l)
            _draw_grasp(vis, K, dist, grasp_cam_l, "GL", (0, 255, 255))
        if off_r is not None:
            grasp_cam_r, grasp_robot_r = _grasp_from_pose(pose, T_rc, off_r)
            _draw_grasp(vis, K, dist, grasp_cam_r, "GR", (255, 200, 0))

    out: dict[str, Any] = {
        "ok": chosen is not None,
        "message": "ok" if chosen is not None else "未检测到传送带 6x6 码",
        "name": chosen["name"] if chosen else "",
        "used_ids": list(chosen["used_ids"]) if chosen else [],
        "reproj_px": float(chosen["reproj_px"]) if chosen else -1.0,
        "origin_cam": list(chosen["origin_cam"]) if chosen else None,
        "origin_robot": list(chosen["origin_robot"]) if chosen else None,
        "R_robot": list(chosen["R_robot"]) if chosen and chosen.get("R_robot") else None,
        "grasp_cam": grasp_cam_l.tolist() if grasp_cam_l is not None else None,
        "grasp_robot": list(grasp_robot_l) if grasp_robot_l is not None else None,
        "grasp_left_robot": list(grasp_robot_l) if grasp_robot_l is not None else None,
        "grasp_right_robot": list(grasp_robot_r) if grasp_robot_r is not None else None,
        "belts": found,
        "save_path": save_path or "",
        "vis_bgr": vis,
    }
    if chosen is not None:
        rob = chosen["origin_robot"]
        cam = chosen["origin_cam"]
        extra = ""
        if grasp_robot_l is not None:
            extra += (
                f" graspL=({grasp_robot_l[0]:.4f},{grasp_robot_l[1]:.4f},{grasp_robot_l[2]:.4f})"
            )
        if grasp_robot_r is not None:
            extra += (
                f" graspR=({grasp_robot_r[0]:.4f},{grasp_robot_r[1]:.4f},{grasp_robot_r[2]:.4f})"
            )
        print(
            f"[belt] {chosen['name']} ids={chosen['used_ids']} "
            f"cam=({cam[0]:.4f},{cam[1]:.4f},{cam[2]:.4f})"
            + (
                f" robot=({rob[0]:.4f},{rob[1]:.4f},{rob[2]:.4f})"
                if rob is not None
                else " robot=无外参"
            )
            + extra
            + f" reproj={chosen['reproj_px']:.2f}px"
        )

    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(save_path, vis)
        out["save_path"] = save_path
    return out
