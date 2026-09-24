"""Head-camera ArUco detection API embedded by the C++ debug runtime."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent


@dataclass(frozen=True)
class Config:
    dictionary: str
    default_side_m: float
    id_to_side_m: dict[int, float]
    pnp_flag: int
    pnp_refine: bool
    axis_ratio: float


_cfg: Config | None = None
_detector = None


def _load_config(path: Path) -> Config:
    with path.open("r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream) or {}
    aruco = root.get("aruco", {})
    pose = root.get("pose", {})
    display = root.get("display", {})

    dictionary = str(aruco.get("dictionary", "DICT_5X5_1000"))
    if not hasattr(cv2.aruco, dictionary):
        raise ValueError(f"Unknown ArUco dictionary: {dictionary}")

    id_to_side_m: dict[int, float] = {}
    for group in aruco.get("sizes", []):
        side = float(group["side_length_m"])
        if side <= 0:
            raise ValueError("ArUco side_length_m must be positive")
        for marker_id in group.get("ids", []):
            id_to_side_m[int(marker_id)] = side

    flag_name = str(pose.get("pnp_flag", "IPPE_SQUARE"))
    flag = getattr(cv2, f"SOLVEPNP_{flag_name}", None)
    if flag is None:
        raise ValueError(f"Unknown solvePnP flag: {flag_name}")

    default_side = float(aruco.get("default_side_length_m", 0.04))
    if default_side <= 0:
        raise ValueError("default_side_length_m must be positive")
    return Config(
        dictionary=dictionary,
        default_side_m=default_side,
        id_to_side_m=id_to_side_m,
        pnp_flag=flag,
        pnp_refine=bool(pose.get("pnp_refine", True)),
        axis_ratio=float(display.get("axis_length_ratio", 0.5)),
    )


def _make_detector(dictionary_id: int):
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    params = (cv2.aruco.DetectorParameters()
              if hasattr(cv2.aruco, "DetectorParameters")
              else cv2.aruco.DetectorParameters_create())
    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(dictionary, params)

    class LegacyDetector:
        def detectMarkers(self, gray):
            return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    return LegacyDetector()


def init(config_path: str | None = None) -> str:
    global _cfg, _detector
    path = (Path(config_path) if config_path else HERE / "config.yaml").expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"ArUco config not found: {path}")
    _cfg = _load_config(path)
    dictionary_id = int(getattr(cv2.aruco, _cfg.dictionary))
    _detector = _make_detector(dictionary_id)
    return (
        f"dict={_cfg.dictionary} default_L={_cfg.default_side_m * 1000:.0f}mm "
        f"id_map={len(_cfg.id_to_side_m)}"
    )


def _object_points(side_m: float) -> np.ndarray:
    half = side_m / 2.0
    return np.array(
        [[-half, half, 0.0], [half, half, 0.0],
         [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )


def _yaw_pitch_roll_deg(rotation: np.ndarray) -> tuple[float, float, float]:
    sy = float(np.hypot(rotation[0, 0], rotation[1, 0]))
    if sy > 1e-9:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = 0.0
    return tuple(float(v) for v in np.degrees([yaw, pitch, roll]))


def detect_frame(
    bgr: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    cam2robot: np.ndarray | None = None,
) -> dict:
    if _cfg is None or _detector is None:
        raise RuntimeError("ArUco engine is not initialized")

    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    dist = np.asarray(dist, dtype=np.float64).reshape(-1)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _detector.detectMarkers(gray)
    vis = bgr.copy()
    markers: list[dict] = []

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)
        for marker_corners, marker_id_array in zip(corners, ids):
            marker_id = int(np.asarray(marker_id_array).reshape(-1)[0])
            side_m = _cfg.id_to_side_m.get(marker_id, _cfg.default_side_m)
            image_points = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
            object_points = _object_points(side_m)
            ok, rvec, tvec = cv2.solvePnP(
                object_points, image_points, K, dist, flags=_cfg.pnp_flag
            )
            if not ok:
                continue
            if _cfg.pnp_refine and hasattr(cv2, "solvePnPRefineLM"):
                rvec, tvec = cv2.solvePnPRefineLM(
                    object_points, image_points, K, dist, rvec, tvec
                )

            projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, dist)
            reproj_px = float(
                np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - image_points) ** 2, axis=1)))
            )
            rotation, _ = cv2.Rodrigues(rvec)
            pose = np.eye(4, dtype=np.float64)
            pose[:3, :3] = rotation
            pose[:3, 3] = tvec.reshape(3)
            yaw_pitch_roll = _yaw_pitch_roll_deg(rotation)

            cv2.drawFrameAxes(vis, K, dist, rvec, tvec, side_m * _cfg.axis_ratio)
            anchor = tuple(int(v) for v in image_points[0])
            cv2.putText(vis, f"id={marker_id} err={reproj_px:.2f}px", anchor,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
            markers.append(
                {
                    "id": marker_id,
                    "side_m": side_m,
                    "reproj_px": reproj_px,
                    "t_m": [float(v) for v in tvec.reshape(3)],
                    # C++ runtime keeps the historic [yaw, pitch, roll] field order.
                    "rpy_deg": list(yaw_pitch_roll),
                    "pose_4x4": pose,
                }
            )

    cv2.putText(vis, f"aruco n={len(markers)}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
    return {"ok": True, "n": len(markers), "markers": markers, "vis": vis}
