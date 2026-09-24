"""Head-camera tray+YOLO overlay API (no RealSense open).

Used by t170c_debug: ArUco 5-marker tray pose → holes 1–36 → associate YOLO
→ sample top-surface z in a small depth window. Does not move the robot.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from online import (
    AppConfig,
    TrayPose,
    estimate_tray_pose,
    get_aruco_dictionary,
    load_config,
    make_detector,
    overlay_tray,
)

HERE = Path(__file__).resolve().parent

CLASS_LABEL_ZH = {
    0: "毛坯",
    1: "半成品",
    2: "成品",
    3: "空孔",
}
# OpenCV Hershey 字体画不出中文，图上必须用 ASCII。
CLASS_LABEL_EN = {
    0: "raw",
    1: "semi",
    2: "fin",
    3: "empty",
}
CLASS_BGR = {
    0: (0, 220, 0),
    1: (0, 220, 220),
    2: (255, 140, 40),
    3: (150, 150, 150),
}

_cfg: AppConfig | None = None
_detector = None
_active_board = "tray"
_tray2_print_m: float | None = None
_tray2_black_m: float | None = None


@dataclass
class _Hole:
    hole_id: int
    row: int
    col: int
    xyz_tray: np.ndarray
    xyz_cam: np.ndarray
    xyz_robot: np.ndarray | None
    uv: tuple[float, float]


def init(config_path: str | Path | None = None) -> str:
    global _cfg, _detector, _tray2_print_m, _tray2_black_m, _active_board
    path = Path(config_path) if config_path else HERE / "config.yaml"
    _cfg = load_config(path)
    _detector = make_detector(get_aruco_dictionary(_cfg.dictionary))
    _active_board = "tray"
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    tray2 = raw.get("tray2") or {}
    _tray2_print_m = float(tray2.get("print_size_m", _cfg.print_size_m))
    _tray2_black_m = float(tray2.get("black_side_m", _cfg.black_side_m))
    return (
        f"dict={_cfg.dictionary} holes={_cfg.hole_rows}x{_cfg.hole_cols} "
        f"pitch={_cfg.hole_pitch_m:.3f}m black={_cfg.black_side_m * 1000:.2f}mm "
        f"tray2_black={_tray2_black_m * 1000:.2f}mm"
    )


def set_active_board(board: str) -> str:
    """料盘1用 tray 的码尺寸。料盘2只换 tray2 的打印边长和黑框。"""
    global _active_board
    _active_board = "tray2" if str(board) == "tray2" else "tray"
    cfg = _board_cfg()
    print(
        f"[tray] 码板={_active_board} 打印={cfg.print_size_m * 1000:.1f}mm "
        f"黑框={cfg.black_side_m * 1000:.2f}mm"
    )
    return _active_board


def _board_cfg() -> AppConfig:
    if _cfg is None:
        raise RuntimeError("tray engine is not initialized")
    if _active_board != "tray2" or _tray2_print_m is None or _tray2_black_m is None:
        return _cfg
    return replace(_cfg, print_size_m=_tray2_print_m, black_side_m=_tray2_black_m)


def _require() -> tuple[AppConfig, Any]:
    if _cfg is None or _detector is None:
        raise RuntimeError("tray engine is not initialized")
    return _board_cfg(), _detector


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
    T = np.asarray(cam2robot, dtype=np.float64).reshape(4, 4)
    return T


def _transform_cam(T_robot_cam: np.ndarray | None, xyz_cam: np.ndarray) -> np.ndarray | None:
    if T_robot_cam is None:
        return None
    p = np.array([xyz_cam[0], xyz_cam[1], xyz_cam[2], 1.0], dtype=np.float64)
    return (T_robot_cam @ p)[:3]


def _cam_from_tray(rvec: np.ndarray, tvec: np.ndarray, xyz_tray: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    return (R @ xyz_tray.reshape(3, 1) + tvec.reshape(3, 1)).reshape(3)


def _rot_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _level_robot_tray(
    T_rc: np.ndarray, rvec: np.ndarray, tvec: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float]:
    """Yaw-only tray in robot: keep camera position, rebuild tray origin with leveled rotation."""
    R_ct, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    t_ct = tvec.reshape(3).astype(np.float64)
    R_rc = T_rc[:3, :3]
    t_rc = T_rc[:3, 3]
    R_rt = R_rc @ R_ct
    t_rt_old = R_rc @ t_ct + t_rc
    z_axis = R_rt[:, 2]
    z_n = z_axis / (np.linalg.norm(z_axis) + 1e-12)
    tilt_deg = float(np.degrees(np.arctan2(np.hypot(z_n[0], z_n[1]), z_n[2])))
    x_axis = R_rt[:, 0]
    yaw = float(np.arctan2(x_axis[1], x_axis[0]))
    R_level = _rot_z(yaw)
    # R_robot_cam := R_level @ R_cam_tray^{-1}，再用它去转 PnP 的盘心，而不是沿用旧外参转出来的原点。
    t_rt = (R_level @ (R_ct.T @ t_ct)) + t_rc
    cam_in_tray = -(R_ct.T @ t_ct)
    print(
        f"[tray] 盘面原点Z {t_rt_old[2]:.3f} -> {t_rt[2]:.3f} "
        f"(Δ {(t_rt[2] - t_rt_old[2]) * 1000:+.1f} mm)  "
        f"cam_z={t_rc[2]:.3f}  above_tray={cam_in_tray[2]:.3f}"
    )
    return R_ct, t_ct, R_level, t_rt, tilt_deg, float(np.degrees(yaw))


def _point_in_tray(R_ct: np.ndarray, t_ct: np.ndarray, xyz_cam: np.ndarray) -> np.ndarray:
    return R_ct.T @ (xyz_cam.reshape(3) - t_ct)


def _project_uv(xyz_cam: np.ndarray, K: np.ndarray, dist: np.ndarray) -> tuple[float, float] | None:
    if xyz_cam[2] <= 1e-4:
        return None
    proj, _ = cv2.projectPoints(
        xyz_cam.reshape(1, 3).astype(np.float32),
        np.zeros(3, dtype=np.float32),
        np.zeros(3, dtype=np.float32),
        K,
        dist,
    )
    uv = proj.reshape(2)
    if not np.all(np.isfinite(uv)):
        return None
    return float(uv[0]), float(uv[1])


def _enumerate_holes(cfg: AppConfig, rvec: np.ndarray, tvec: np.ndarray, K, dist, T_rc):
    """孔行列绑定料盘 ArUco 系：+X 前=row1，-X 后=row6，-Y 右=col1，+Y 左=col6。
    与机器人远近无关；抓取姿态远近在 C++ 里按基座 X 再排。"""
    centers = cfg.hole_centers_tray().reshape(cfg.hole_rows, cfg.hole_cols, 3)
    holes: list[_Hole] = []
    for front_i in range(cfg.hole_rows):
        i = cfg.hole_rows - 1 - front_i
        for j in range(cfg.hole_cols):
            xyz_tray = centers[i, j]
            xyz_cam = _cam_from_tray(rvec, tvec, xyz_tray)
            uv = _project_uv(xyz_cam, K, dist)
            if uv is None:
                continue
            holes.append(
                _Hole(
                    hole_id=front_i * cfg.hole_cols + j + 1,
                    row=front_i + 1,
                    col=j + 1,
                    xyz_tray=xyz_tray,
                    xyz_cam=xyz_cam,
                    xyz_robot=_transform_cam(T_rc, xyz_cam),
                    uv=uv,
                )
            )
    return holes


def _sample_top_xyz(
    depth_m: np.ndarray | None,
    uv: tuple[float, float],
    xyz_cam: np.ndarray,
    K: np.ndarray,
    radius_m: float,
) -> tuple[np.ndarray | None, int]:
    """Shallowest 10% depth in a small window → camera-frame 3D point."""
    if depth_m is None:
        return None, 0
    h, w = depth_m.shape[:2]
    z_cam = float(xyz_cam[2])
    if z_cam <= 1e-4:
        return None, 0
    fx = float(K[0, 0])
    radius_px = max(6.0, fx * radius_m / z_cam)
    uu, vv = uv
    x0 = max(0, int(np.floor(uu - radius_px)))
    x1 = min(w, int(np.ceil(uu + radius_px)) + 1)
    y0 = max(0, int(np.floor(vv - radius_px)))
    y1 = min(h, int(np.ceil(vv + radius_px)) + 1)
    if x1 <= x0 or y1 <= y0:
        return None, 0
    yy, xx = np.ogrid[y0:y1, x0:x1]
    disk = (xx - uu) ** 2 + (yy - vv) ** 2 <= radius_px ** 2
    patch = depth_m[y0:y1, x0:x1][disk]
    valid = np.isfinite(patch) & (patch >= 0.05) & (patch <= 2.0)
    vals = patch[valid].astype(np.float64)
    if vals.size < 8:
        return None, int(vals.size)
    cut = np.percentile(vals, 10.0)
    shallow = vals[vals <= cut]
    z_s = float(np.median(shallow if shallow.size else vals))
    xyz = np.array(
        [
            (uu - float(K[0, 2])) * z_s / fx,
            (vv - float(K[1, 2])) * z_s / float(K[1, 1]),
            z_s,
        ],
        dtype=np.float64,
    )
    return xyz, int(vals.size)


def _mean_rotation_svd(Rs: list[np.ndarray]) -> np.ndarray:
    """Chordal mean：ΣR 的 SVD 投影回 SO(3)。"""
    M = np.zeros((3, 3), dtype=np.float64)
    for R in Rs:
        M += np.asarray(R, dtype=np.float64).reshape(3, 3)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0.0:
        U = U.copy()
        U[:, 2] *= -1.0
        R = U @ Vt
    return R


def _geodesic_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    c = np.clip((np.trace(R_a.T @ R_b) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def _reject_leveled_outliers(
    Rs: list[np.ndarray], ts: list[np.ndarray]
) -> tuple[list[np.ndarray], list[np.ndarray], int]:
    """丢掉相对中位数偏差过大的帧，避免坏 PnP 把平均拉飞。至少留 2 帧。"""
    n = len(Rs)
    if n <= 2:
        return Rs, ts, 0
    t = np.stack(ts, axis=0)
    t_med = np.median(t, axis=0)
    d = np.linalg.norm(t - t_med, axis=1)
    mad_t = float(np.median(np.abs(d - np.median(d))))
    t_lim = max(0.010, 3.0 * 1.4826 * mad_t)

    R_ref = _mean_rotation_svd(Rs)
    ang = np.array([_geodesic_deg(R_ref, R) for R in Rs], dtype=np.float64)
    mad_a = float(np.median(np.abs(ang - np.median(ang))))
    a_lim = max(2.0, 3.0 * 1.4826 * mad_a)

    keep = (d <= t_lim) & (ang <= a_lim)
    n_keep = int(np.count_nonzero(keep))
    if n_keep < 2:
        return Rs, ts, 0
    dropped = n - n_keep
    if dropped:
        for i, ok in enumerate(keep):
            if not ok:
                print(
                    f"[tray] 丢掉第{i + 1}帧  Δt={d[i] * 1000:.1f}mm "
                    f"(限{t_lim * 1000:.1f})  ΔR={ang[i]:.2f}deg (限{a_lim:.1f})"
                )
    Rs_k = [R for R, ok in zip(Rs, keep) if ok]
    ts_k = [ti for ti, ok in zip(ts, keep) if ok]
    return Rs_k, ts_k, dropped


def _T_from_Rt(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def _rtvec_from_T(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return rvec.reshape(3), T[:3, 3].copy()


def _estimate_leveled_T(
    bgr: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    T_rc: np.ndarray | None,
) -> dict[str, Any] | None:
    cfg, detector = _require()
    gray = cv2.cvtColor(np.ascontiguousarray(bgr), cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    tray = estimate_tray_pose(corners, ids, cfg, K, dist)
    if tray is None or T_rc is None:
        return None
    _R_ct, _t_ct, R_level, t_level, tilt_deg, yaw_deg = _level_robot_tray(
        T_rc, tray.rvec, tray.tvec
    )
    return {
        "tray": tray,
        "R_level": R_level,
        "t_level": t_level,
        "tilt_deg": tilt_deg,
        "yaw_deg": yaw_deg,
    }


def detect_and_annotate(
    bgr: np.ndarray,
    depth_m: np.ndarray | None,
    K: np.ndarray,
    dist: np.ndarray | None,
    cam2robot: np.ndarray | None,
    detections: list[dict[str, Any]] | None,
    save_path: str | None = None,
    associate_max_m: float = 0.03,
    R_level_override: np.ndarray | None = None,
    t_level_override: np.ndarray | None = None,
    fuse_ok: int = 0,
    fuse_n: int = 0,
) -> dict[str, Any]:
    cfg, detector = _require()
    K = _as_K(K)
    dist = _as_dist(dist)
    T_rc = _as_T(cam2robot)
    bgr = np.ascontiguousarray(bgr)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    tray = estimate_tray_pose(corners, ids, cfg, K, dist)

    vis = bgr.copy()
    n_ids = 0 if ids is None else int(len(ids.flatten()))
    used: list[int] = []
    holes_out: list[dict[str, Any]] = []

    have_override = R_level_override is not None and t_level_override is not None
    if tray is None and have_override and T_rc is not None:
        T_rt = _T_from_Rt(
            np.asarray(R_level_override, dtype=np.float64),
            np.asarray(t_level_override, dtype=np.float64),
        )
        T_ct = np.linalg.inv(T_rc) @ T_rt
        rvec_s, tvec_s = _rtvec_from_T(T_ct)
        tray = TrayPose(
            rvec=rvec_s,
            tvec=tvec_s,
            reproj_px=-1.0,
            used_ids=[],
            corners_by_id={},
        )

    if tray is None:
        cv2.putText(
            vis,
            f"tray=FAIL markers={n_ids} (need>={cfg.min_markers})",
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        if save_path:
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(save_path, vis)
        print(f"[tray] FAIL markers={n_ids} save={save_path or '-'}")
        return {
            "ok": False,
            "message": f"料盘位姿失败，看到 {n_ids} 个码",
            "used_ids": [],
            "reproj_px": -1.0,
            "save_path": save_path or "",
            "holes": [],
            "origin_xyz": None,
            "vis_bgr": vis,
        }

    used = list(tray.used_ids)
    overlay_tray(vis, tray, cfg, K, dist)
    holes = _enumerate_holes(cfg, tray.rvec, tray.tvec, K, dist, T_rc)
    radius_m = min(0.012, cfg.hole_diameter_m * 0.35)
    R_ct = t_ct = R_level = t_level = None
    tilt_deg = float("nan")
    yaw_deg = float("nan")
    if T_rc is not None:
        R_ct, t_ct, R_level, t_level, tilt_deg, yaw_deg = _level_robot_tray(
            T_rc, tray.rvec, tray.tvec
        )
    if have_override:
        R_level = np.asarray(R_level_override, dtype=np.float64).reshape(3, 3)
        t_level = np.asarray(t_level_override, dtype=np.float64).reshape(3)

    dets: list[dict[str, Any]] = []
    for raw in detections or []:
        xyz = np.array(raw.get("xyz_cam", [np.nan, np.nan, np.nan]), dtype=np.float64)
        if not np.all(np.isfinite(xyz)):
            continue
        xyz_r = _transform_cam(T_rc, xyz)
        dets.append(
            {
                "class_id": int(raw.get("class_id", -1)),
                "class_name": str(raw.get("class_name", "")),
                "conf": float(raw.get("conf", 0.0)),
                "xyz_cam": xyz,
                "xyz_robot": xyz_r,
            }
        )

    claimed: set[int] = set()
    for hole in holes:
        best = None
        best_d = 1e9
        use_robot = hole.xyz_robot is not None
        hx, hy = (hole.xyz_robot[0], hole.xyz_robot[1]) if use_robot else (hole.xyz_cam[0], hole.xyz_cam[1])
        for di, det in enumerate(dets):
            if di in claimed:
                continue
            src = det["xyz_robot"] if use_robot and det["xyz_robot"] is not None else det["xyz_cam"]
            dxy = float(np.hypot(src[0] - hx, src[1] - hy))
            if dxy < best_d:
                best_d = dxy
                best = (di, det)
        cls_id = -1
        cls_name = ""
        conf = 0.0
        dxy = -1.0
        if best is not None and best_d <= associate_max_m:
            claimed.add(best[0])
            cls_id = int(best[1]["class_id"])
            cls_name = str(best[1]["class_name"] or CLASS_LABEL_ZH.get(cls_id, ""))
            conf = float(best[1]["conf"])
            dxy = best_d

        xyz_depth, n_depth = _sample_top_xyz(
            None if depth_m is None else np.asarray(depth_m),
            hole.uv,
            hole.xyz_cam,
            K,
            radius_m,
        )
        tray_z = float(hole.xyz_robot[2]) if hole.xyz_robot is not None else None
        top_z = None
        if xyz_depth is not None:
            robot_raw = _transform_cam(T_rc, xyz_depth)
            if robot_raw is not None:
                top_z = float(robot_raw[2])
        height_on_tray = None
        if xyz_depth is not None and R_ct is not None:
            height_on_tray = float(_point_in_tray(R_ct, t_ct, xyz_depth)[2])
        xyz_level = None
        if R_level is not None:
            xyz_level = R_level @ hole.xyz_tray.reshape(3) + t_level
        tray_z_level = float(xyz_level[2]) if xyz_level is not None else None
        top_z_level = None
        if tray_z_level is not None and height_on_tray is not None:
            top_z_level = tray_z_level + height_on_tray
        color = CLASS_BGR.get(cls_id, (255, 255, 255))
        u, v = int(round(hole.uv[0])), int(round(hole.uv[1]))
        cv2.circle(vis, (u, v), 11, color, 2, cv2.LINE_AA)
        label = f"{hole.hole_id}"
        if cls_id >= 0:
            label += f" {CLASS_LABEL_EN.get(cls_id, cls_name or f'c{cls_id}')}"
        z_txt = ""
        if top_z_level is not None:
            z_txt = f" z={top_z_level:.3f}"
        elif top_z is not None:
            z_txt = f" z={top_z:.3f}"
        elif tray_z_level is not None:
            z_txt = f" 盘={tray_z_level:.3f}"
        elif tray_z is not None:
            z_txt = f" 盘={tray_z:.3f}"
        cv2.putText(
            vis,
            label + z_txt,
            (u + 8, v - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            color,
            1,
            cv2.LINE_AA,
        )
        holes_out.append(
            {
                "id": hole.hole_id,
                "row": hole.row,
                "col": hole.col,
                "x": float(hole.xyz_robot[0]) if hole.xyz_robot is not None else float(hole.xyz_cam[0]),
                "y": float(hole.xyz_robot[1]) if hole.xyz_robot is not None else float(hole.xyz_cam[1]),
                "x_level": float(xyz_level[0]) if xyz_level is not None else float("nan"),
                "y_level": float(xyz_level[1]) if xyz_level is not None else float("nan"),
                "tray_z": tray_z if tray_z is not None else float("nan"),
                "tray_z_level": tray_z_level if tray_z_level is not None else float("nan"),
                "top_z": top_z if top_z is not None else float("nan"),
                "top_z_level": top_z_level if top_z_level is not None else float("nan"),
                "height_on_tray": height_on_tray if height_on_tray is not None else float("nan"),
                "class_id": cls_id,
                "class_name": CLASS_LABEL_ZH.get(cls_id, cls_name),
                "conf": conf,
                "dxy": dxy,
                "depth_pts": n_depth,
                "in_robot": hole.xyz_robot is not None,
            }
        )

    for det in dets:
        uv = _project_uv(det["xyz_cam"], K, dist)
        if uv is None:
            continue
        color = CLASS_BGR.get(int(det["class_id"]), (255, 255, 255))
        cv2.drawMarker(
            vis,
            (int(round(uv[0])), int(round(uv[1]))),
            color,
            cv2.MARKER_TILTED_CROSS,
            14,
            2,
            cv2.LINE_AA,
        )

    warn = ""
    if len(used) < 3:
        warn = f" only {len(used)} markers, far holes may drift"
    fuse_txt = f" fuse={fuse_ok}/{fuse_n}" if fuse_n > 0 else ""
    tilt_txt = f" tilt={tilt_deg:.1f}deg" if np.isfinite(tilt_deg) else ""
    cv2.putText(
        vis,
        f"tray=OK ids={used} n={len(used)}/5 reproj={tray.reproj_px:.2f}px{tilt_txt}{fuse_txt}{warn}",
        (8, vis.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255) if len(used) >= 3 else (0, 165, 255),
        1,
        cv2.LINE_AA,
    )
    legend_y = 52
    for cid, name in CLASS_LABEL_EN.items():
        cv2.putText(
            vis,
            f"{cid} {name}",
            (vis.shape[1] - 110, legend_y + cid * 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            CLASS_BGR[cid],
            1,
            cv2.LINE_AA,
        )

    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(save_path, vis)

    _print_holes_table(holes_out, used, float(tray.reproj_px), save_path, tilt_deg)
    msg = "ok"
    if warn:
        msg += " " + warn.strip()
    if np.isfinite(tilt_deg):
        msg += f" tilt={tilt_deg:.1f}deg"
    if fuse_n > 0:
        msg += f" fuse={fuse_ok}/{fuse_n}"
    return {
        "ok": True,
        "message": msg,
        "used_ids": used,
        "reproj_px": float(tray.reproj_px),
        "tilt_deg": float(tilt_deg) if np.isfinite(tilt_deg) else float("nan"),
        "yaw_deg": float(yaw_deg) if np.isfinite(yaw_deg) else float("nan"),
        "save_path": save_path or "",
        "holes": holes_out,
        "origin_xyz": (
            [float(t_level[0]), float(t_level[1]), float(t_level[2])]
            if t_level is not None
            else None
        ),
        "vis_bgr": vis,
    }


def _fmt_z(v: float | None) -> str:
    if v is None or not np.isfinite(v):
        return "   ---"
    return f"{v:7.3f}"


def _print_holes_table(
    holes: list[dict[str, Any]],
    used_ids: list[int],
    reproj_px: float,
    save_path: str | None,
    tilt_deg: float | None = None,
) -> None:
    tilt_s = f" tilt={tilt_deg:.2f}deg" if tilt_deg is not None and np.isfinite(tilt_deg) else ""
    print(
        f"[tray] ids={used_ids} n={len(used_ids)}/5 reproj={reproj_px:.2f}px{tilt_s}  "
        f"save={save_path or '-'}"
    )
    print(
        f"{'孔':>3} {'行':>2} {'列':>2} {'类别':<8} {'基座X':>8} {'基座Y':>8} "
        f"{'盘面Z':>8} {'水平盘Z':>8} {'盘上高':>8} {'水平顶Z':>8} {'原顶Z':>8}"
    )
    for h in holes:
        name = str(h.get("class_name") or "")
        if h.get("class_id", -1) < 0:
            name = "-"
        print(
            f"{int(h['id']):3d} {int(h['row']):2d} {int(h['col']):2d} {name:<8} "
            f"{float(h['x']):8.3f} {float(h['y']):8.3f} "
            f"{_fmt_z(h.get('tray_z'))} {_fmt_z(h.get('tray_z_level'))} "
            f"{_fmt_z(h.get('height_on_tray'))} {_fmt_z(h.get('top_z_level'))} "
            f"{_fmt_z(h.get('top_z'))}"
        )
    level_zs = [float(h["tray_z_level"]) for h in holes if np.isfinite(h.get("tray_z_level", np.nan))]
    raw_zs = [float(h["tray_z"]) for h in holes if np.isfinite(h.get("tray_z", np.nan))]
    if level_zs and raw_zs:
        print(
            f"[tray] 原盘面Z跨度 {(max(raw_zs) - min(raw_zs)) * 1000:.1f} mm  "
            f"水平约束后 {(max(level_zs) - min(level_zs)) * 1000:.1f} mm"
        )


def fuse_and_annotate(
    bgrs: list[Any],
    depth_m: np.ndarray | None,
    K: np.ndarray,
    dist: np.ndarray | None,
    cam2robot: np.ndarray | None,
    detections: list[dict[str, Any]] | None,
    save_path: str | None = None,
    associate_max_m: float = 0.03,
) -> dict[str, Any]:
    """多帧扶平 T_robot_tray：丢掉大偏差帧后旋转 SVD 平均、平移取均值；YOLO/可视化用最后一帧。"""
    if not bgrs:
        return {
            "ok": False,
            "message": "没有可融合的帧",
            "used_ids": [],
            "reproj_px": -1.0,
            "save_path": save_path or "",
            "holes": [],
            "origin_xyz": None,
            "vis_bgr": np.zeros((480, 640, 3), dtype=np.uint8),
        }
    K = _as_K(K)
    dist = _as_dist(dist)
    T_rc = _as_T(cam2robot)
    Rs: list[np.ndarray] = []
    ts: list[np.ndarray] = []
    for raw in bgrs:
        bgr = np.ascontiguousarray(raw)
        est = _estimate_leveled_T(bgr, K, dist, T_rc)
        if est is None:
            continue
        Rs.append(est["R_level"])
        ts.append(np.asarray(est["t_level"], dtype=np.float64).reshape(3))
    last = np.ascontiguousarray(bgrs[-1])
    if not Rs:
        return detect_and_annotate(
            last, depth_m, K, dist, cam2robot, detections, save_path, associate_max_m
        )
    n_ok = len(Rs)
    Rs, ts, n_drop = _reject_leveled_outliers(Rs, ts)
    R_avg = _mean_rotation_svd(Rs)
    t_avg = np.mean(np.stack(ts, axis=0), axis=0)
    drop_txt = f" 丢{n_drop}帧" if n_drop else ""
    print(
        f"[tray] 多帧融合 {len(Rs)}/{len(bgrs)}（解算{n_ok}{drop_txt}）  "
        f"t_avg=({t_avg[0]:.4f},{t_avg[1]:.4f},{t_avg[2]:.4f})"
    )
    return detect_and_annotate(
        last,
        depth_m,
        K,
        dist,
        cam2robot,
        detections,
        save_path,
        associate_max_m,
        R_level_override=R_avg,
        t_level_override=t_avg,
        fuse_ok=len(Rs),
        fuse_n=len(bgrs),
    )


def default_save_path(project_root: str) -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    return str(Path(project_root) / "picture_debug" / "head" / f"tray_yolo_{ts}.jpg")
