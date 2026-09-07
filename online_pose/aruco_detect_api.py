"""给 orch_hw 调用：已有 RGB + 相机内参，按码 ID 查边长再解 PnP。

不打开 RealSense（相机由 C++ 占用）。配置见同目录 config.yaml。
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from aruco_pose_online import (
    AppConfig,
    estimate_poses,
    get_aruco_dictionary,
    load_config,
    make_detector,
    overlay_poses,
    pose_to_rpy_tmm,
)

HERE = Path(__file__).resolve().parent

_cfg: AppConfig | None = None
_detector = None


def init(config_path: str | None = None) -> str:
    global _cfg, _detector
    path = Path(config_path) if config_path else HERE / "config.yaml"
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"ArUco 配置不存在: {path}")
    _cfg = load_config(path)
    _detector = make_detector(get_aruco_dictionary(_cfg.dictionary))
    n_map = len(_cfg.id_to_side_m)
    return (
        f"dict={_cfg.dictionary} default_L={_cfg.default_side_m * 1000:.0f}mm "
        f"id_map={n_map} pnp={_cfg.pnp_flag}"
    )


def detect_frame(
    bgr: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    cam2robot: np.ndarray | None = None,
) -> dict:
    if _cfg is None or _detector is None:
        raise RuntimeError("ArUco 未 init")

    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    dist = np.asarray(dist, dtype=np.float64).reshape(-1)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _detector.detectMarkers(gray)
    poses = estimate_poses(corners, ids, _cfg, K, dist)

    vis = bgr.copy()
    if ids is not None and len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)
    overlay_poses(vis, poses, K, dist, _cfg.axis_ratio, cam2robot=cam2robot)
    cv2.putText(
        vis,
        f"aruco n={len(poses)}",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    markers: list[dict] = []
    for p in poses:
        R, _ = cv2.Rodrigues(p.rvec)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = p.tvec.reshape(3)
        rpy, _, _ = pose_to_rpy_tmm(p.rvec, p.tvec)
        markers.append(
            {
                "id": int(p.marker_id),
                "side_m": float(p.side_m),
                "reproj_px": float(p.reproj_px),
                "t_m": [float(p.tvec[0]), float(p.tvec[1]), float(p.tvec[2])],
                "rpy_deg": [float(rpy[0]), float(rpy[1]), float(rpy[2])],
                "pose_4x4": T,
            }
        )
    return {"ok": True, "n": len(markers), "markers": markers, "vis": vis}
