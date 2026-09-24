#!/usr/bin/env python3
"""
RealSense 在线料盘 6D 位姿：用盘上 5 个 5×5 ArUco（ID 0–4）联合 PnP。

布局（俯视，码面朝上）：
        前 (+X)
  ID0 左上           ID1 右上
左(+Y)      ID2 中心            右(-Y)
  ID3 左下           ID4 右下
        后 (-X)
+Z 上，垂直码面向外。原点在中心码中心。

打印最外边 40 mm（含白边）。OpenCV 检测角点是黑框外角，黑框边长固定 31.2 mm。

用法:
  python online.py
  python online.py --config config.yaml
  python online.py --list-cameras
  python online.py --serial <序列号>

按键: q / ESC 退出 | s 保存截图
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

HERE = Path(__file__).resolve().parent
PNP_FLAGS = {
    "SQPNP": cv2.SOLVEPNP_SQPNP,
    "ITERATIVE": cv2.SOLVEPNP_ITERATIVE,
    "IPPE": cv2.SOLVEPNP_IPPE,
}

# 码局部：+X 右、+Y 上（码图案上方）、+Z 向外。
# 贴码时码的「上」朝料盘 +X（前）→ 码系到料盘系：
#   料盘X = 码Y，料盘Y = -码X，料盘Z = 码Z。
R_MARKER_IN_TRAY = np.array(
    [
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

SLOT_KEYS = ("front_left", "front_right", "center", "rear_left", "rear_right")


@dataclass
class AppConfig:
    dictionary: str
    print_size_m: float
    black_side_m: float
    bit_cells: int
    black_border_cells: int
    white_border_cells: int
    pitch_xy_m: float
    slot_to_id: dict[str, int]
    hole_rows: int
    hole_cols: int
    hole_pitch_m: float
    hole_diameter_m: float
    hole_offset_xy_m: tuple[float, float]
    pnp_flag: str
    pnp_refine: bool
    min_markers: int
    cam_width: int
    cam_height: int
    cam_fps: int
    cam_serial: str
    axis_length_m: float
    print_every: int
    save_dir: Path
    window_name: str

    @property
    def id_to_slot(self) -> dict[int, str]:
        return {mid: slot for slot, mid in self.slot_to_id.items()}

    def marker_center_tray(self, slot: str) -> np.ndarray:
        h = self.pitch_xy_m * 0.5
        return {
            "front_left": np.array([h, h, 0.0], dtype=np.float64),
            "front_right": np.array([h, -h, 0.0], dtype=np.float64),
            "center": np.array([0.0, 0.0, 0.0], dtype=np.float64),
            "rear_left": np.array([-h, h, 0.0], dtype=np.float64),
            "rear_right": np.array([-h, -h, 0.0], dtype=np.float64),
        }[slot]

    def hole_centers_tray(self) -> np.ndarray:
        """6×6 孔心，在料盘 XOY。行沿 +X（前），列沿 +Y（左），网格默认关于原点对称。"""
        n_x, n_y = self.hole_rows, self.hole_cols
        ox, oy = self.hole_offset_xy_m
        xs = (np.arange(n_x, dtype=np.float64) - (n_x - 1) * 0.5) * self.hole_pitch_m + ox
        ys = (np.arange(n_y, dtype=np.float64) - (n_y - 1) * 0.5) * self.hole_pitch_m + oy
        xx, yy = np.meshgrid(xs, ys, indexing="ij")
        pts = np.stack([xx.reshape(-1), yy.reshape(-1), np.zeros(n_x * n_y)], axis=1)
        return pts


@dataclass
class RealSenseDevice:
    serial: str
    name: str

    @property
    def label(self) -> str:
        short = self.serial if len(self.serial) <= 14 else f"{self.serial[:10]}..."
        return f"{self.name} [{short}]"


@dataclass
class TrayPose:
    rvec: np.ndarray
    tvec: np.ndarray
    reproj_px: float
    used_ids: list[int]
    corners_by_id: dict[int, np.ndarray]


def load_config(path: Path) -> AppConfig:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    aruco = raw.get("aruco", {})
    tray = raw.get("tray", {})
    pose = raw.get("pose", {})
    camera = raw.get("camera", {})
    display = raw.get("display", {})

    ids_raw = tray.get("ids", {})
    slot_to_id: dict[str, int] = {}
    for key in SLOT_KEYS:
        if key not in ids_raw:
            raise ValueError(f"tray.ids 缺少槽位 {key}")
        slot_to_id[key] = int(ids_raw[key])
    if len(set(slot_to_id.values())) != 5:
        raise ValueError("五个槽位的 ID 必须互不相同")

    print_size = float(tray.get("print_size_m", 0.04))
    bit_cells = int(tray.get("bit_cells", 5))
    black_border = int(tray.get("black_border_cells", 1))
    white_border = int(tray.get("white_border_cells", 1))
    if tray.get("black_side_m") is not None:
        black_side = float(tray["black_side_m"])
    else:
        black_cells = bit_cells + 2 * black_border
        total_cells = black_cells + 2 * white_border
        black_side = print_size * black_cells / total_cells
    pitch = float(tray.get("pitch_xy_m", 0.30))
    holes = tray.get("holes", {})
    hole_rows = int(holes.get("rows", 6))
    hole_cols = int(holes.get("cols", 6))
    hole_pitch = float(holes.get("pitch_m", 0.075))
    hole_diam = float(holes.get("diameter_m", 0.038))
    hole_ox = float(holes.get("offset_x_m", 0.0))
    hole_oy = float(holes.get("offset_y_m", 0.0))
    if print_size <= 0 or black_side <= 0 or pitch <= 0:
        raise ValueError("print_size_m / black_side_m / pitch_xy_m 必须 > 0")
    if bit_cells <= 0 or black_border < 0 or white_border < 0:
        raise ValueError("格数配置无效")
    if hole_rows < 1 or hole_cols < 1 or hole_pitch <= 0 or hole_diam <= 0:
        raise ValueError("holes 配置无效")

    save_dir = Path(display.get("save_dir", "captures"))
    if not save_dir.is_absolute():
        save_dir = path.parent / save_dir

    min_markers = int(pose.get("min_markers", 1))
    if min_markers < 1:
        min_markers = 1

    return AppConfig(
        dictionary=str(aruco.get("dictionary", "DICT_5X5_1000")),
        print_size_m=print_size,
        black_side_m=black_side,
        bit_cells=bit_cells,
        black_border_cells=black_border,
        white_border_cells=white_border,
        pitch_xy_m=pitch,
        slot_to_id=slot_to_id,
        hole_rows=hole_rows,
        hole_cols=hole_cols,
        hole_pitch_m=hole_pitch,
        hole_diameter_m=hole_diam,
        hole_offset_xy_m=(hole_ox, hole_oy),
        pnp_flag=str(pose.get("pnp_flag", "SQPNP")),
        pnp_refine=bool(pose.get("pnp_refine", True)),
        min_markers=min_markers,
        cam_width=int(camera.get("width", 1280)),
        cam_height=int(camera.get("height", 720)),
        cam_fps=int(camera.get("fps", 30)),
        cam_serial=str(camera.get("serial", "") or ""),
        axis_length_m=float(display.get("axis_length_m", 0.08)),
        print_every=int(display.get("print_pose_every_n_frames", 15)),
        save_dir=save_dir,
        window_name=str(display.get("window_name", "board_yf100_aruco")),
    )


def get_aruco_dictionary(name: str) -> cv2.aruco.Dictionary:
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"未知 ArUco 字典: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_detector(dictionary: cv2.aruco.Dictionary) -> cv2.aruco.ArucoDetector:
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary, params)


def marker_corners_local(black_side_m: float) -> np.ndarray:
    h = black_side_m * 0.5
    return np.array(
        [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
        dtype=np.float64,
    )


def marker_corners_in_tray(cfg: AppConfig, marker_id: int) -> np.ndarray:
    slot = cfg.id_to_slot[marker_id]
    local = marker_corners_local(cfg.black_side_m)
    center = cfg.marker_center_tray(slot)
    return local @ R_MARKER_IN_TRAY.T + center


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


def pose_to_rpy_tmm(
    rvec: np.ndarray, tvec: np.ndarray
) -> tuple[tuple[float, float, float], np.ndarray, float]:
    R, _ = cv2.Rodrigues(rvec)
    rpy = rotation_matrix_to_euler_zyx_deg(R)
    t_mm = tvec.reshape(3) * 1000.0
    dist_mm = float(np.linalg.norm(tvec) * 1000.0)
    return rpy, t_mm, dist_mm


def mean_reprojection_error(
    obj_pts: np.ndarray,
    img_pts: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
) -> float:
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - img_pts.reshape(-1, 2), axis=1)
    return float(np.mean(err))


def estimate_tray_pose(
    corners,
    ids,
    cfg: AppConfig,
    K: np.ndarray,
    dist: np.ndarray,
) -> TrayPose | None:
    if ids is None or len(corners) == 0:
        return None
    known = cfg.id_to_slot
    obj_list: list[np.ndarray] = []
    img_list: list[np.ndarray] = []
    used: list[int] = []
    corners_by_id: dict[int, np.ndarray] = {}
    for i, mid_raw in enumerate(ids.flatten()):
        mid = int(mid_raw)
        if mid not in known:
            continue
        obj_list.append(marker_corners_in_tray(cfg, mid))
        img_list.append(corners[i].reshape(4, 2).astype(np.float64))
        used.append(mid)
        corners_by_id[mid] = corners[i]
    if len(used) < cfg.min_markers:
        return None
    obj_pts = np.concatenate(obj_list, axis=0).astype(np.float32)
    img_pts = np.concatenate(img_list, axis=0).astype(np.float32)
    flag = PNP_FLAGS.get(cfg.pnp_flag, cv2.SOLVEPNP_SQPNP)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist, flags=flag)
    if not ok:
        return None
    if cfg.pnp_refine and hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(obj_pts, img_pts, K, dist, rvec, tvec)
    err = mean_reprojection_error(obj_pts, img_pts, rvec, tvec, K, dist)
    return TrayPose(
        rvec=rvec.reshape(3),
        tvec=tvec.reshape(3),
        reproj_px=err,
        used_ids=sorted(used),
        corners_by_id=corners_by_id,
    )


def draw_pose_text(vis: np.ndarray, lines: list[str], origin: tuple[int, int], color) -> None:
    x0, y0 = origin
    for j, line in enumerate(lines):
        cv2.putText(
            vis, line, (x0, y0 + j * 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA,
        )


def draw_detected_markers_thin(vis: np.ndarray, corners, ids, tray_ids: set[int]) -> None:
    if ids is None or len(corners) == 0:
        return
    for i, mid in enumerate(ids.flatten()):
        mid = int(mid)
        pts = corners[i].reshape(-1, 2).astype(np.int32)
        color = (0, 255, 0) if mid in tray_ids else (80, 80, 80)
        cv2.polylines(vis, [pts], True, color, 1, cv2.LINE_AA)
        cv2.circle(vis, (int(pts[0, 0]), int(pts[0, 1])), 2, (0, 0, 255), -1, cv2.LINE_AA)
        tx = int(pts[:, 0].min())
        ty = int(max(pts[:, 1].min() - 3, 10))
        cv2.putText(
            vis, str(mid), (tx, ty),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA,
        )


def _project_pts(
    pts_tray: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, K, dist
) -> np.ndarray:
    proj, _ = cv2.projectPoints(pts_tray.astype(np.float32), rvec, tvec, K, dist)
    return proj.reshape(-1, 2)


def hole_circle_pts_tray(center: np.ndarray, radius_m: float, n: int = 48) -> np.ndarray:
    theta = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    pts = np.stack(
        [
            center[0] + radius_m * np.cos(theta),
            center[1] + radius_m * np.sin(theta),
            np.full(n, center[2]),
        ],
        axis=1,
    )
    return pts


def _in_front_of_camera(pts_tray: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec)
    cam = pts_tray @ R.T + tvec.reshape(1, 3)
    return cam[:, 2] > 1e-4


def overlay_holes(
    vis: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    cfg: AppConfig,
    K,
    dist,
) -> None:
    centers = cfg.hole_centers_tray()
    radius = cfg.hole_diameter_m * 0.5
    h, w = vis.shape[:2]
    front = _in_front_of_camera(centers, rvec, tvec)
    if not np.any(front):
        return
    center_pix = _project_pts(centers, rvec, tvec, K, dist)
    color_edge = (255, 80, 255)
    color_dot = (255, 220, 255)
    for i, c in enumerate(centers):
        if not front[i]:
            continue
        cx, cy = center_pix[i]
        if not (np.isfinite(cx) and np.isfinite(cy)):
            continue
        ring = hole_circle_pts_tray(c, radius)
        if not np.all(_in_front_of_camera(ring, rvec, tvec)):
            continue
        poly = _project_pts(ring, rvec, tvec, K, dist)
        if not np.all(np.isfinite(poly)):
            continue
        pix = np.round(poly).astype(np.int32)
        cv2.polylines(vis, [pix], True, color_edge, 1, cv2.LINE_AA)
        ix, iy = int(round(cx)), int(round(cy))
        if 0 <= ix < w and 0 <= iy < h:
            cv2.circle(vis, (ix, iy), 2, color_dot, -1, cv2.LINE_AA)


def overlay_tray(
    vis: np.ndarray,
    tray: TrayPose | None,
    cfg: AppConfig,
    K,
    dist,
) -> np.ndarray:
    if tray is None:
        return vis
    rvec = tray.rvec.reshape(3, 1)
    tvec = tray.tvec.reshape(3, 1)
    cv2.drawFrameAxes(vis, K, dist, rvec, tvec, cfg.axis_length_m, 2)

    slot_pts = np.stack(
        [cfg.marker_center_tray(s) for s in ("front_left", "front_right", "rear_right", "rear_left")],
        axis=0,
    )
    quad = _project_pts(slot_pts, rvec, tvec, K, dist).astype(np.int32)
    cv2.polylines(vis, [quad], True, (0, 200, 255), 1, cv2.LINE_AA)
    overlay_holes(vis, rvec, tvec, cfg, K, dist)

    axis_pts = np.array(
        [
            [0.0, 0.0, 0.0],
            [cfg.axis_length_m, 0.0, 0.0],
            [0.0, cfg.axis_length_m, 0.0],
            [0.0, 0.0, cfg.axis_length_m],
        ],
        dtype=np.float64,
    )
    pix = _project_pts(axis_pts, rvec, tvec, K, dist).astype(int)
    labels = ("O", "F", "L", "U")
    colors = ((255, 255, 255), (0, 0, 255), (0, 255, 0), (255, 0, 0))
    for (x, y), lab, col in zip(pix, labels, colors):
        cv2.putText(
            vis, lab, (int(x) + 3, int(y) - 3),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA,
        )

    rpy, t_mm, dist_mm = pose_to_rpy_tmm(tray.rvec, tray.tvec)
    draw_pose_text(
        vis,
        [
            f"tray n={len(tray.used_ids)}/5 ids={tray.used_ids}",
            f"t(mm)=({t_mm[0]:+.0f},{t_mm[1]:+.0f},{t_mm[2]:+.0f}) dist={dist_mm:.0f}",
            f"rpy=({rpy[0]:+.1f},{rpy[1]:+.1f},{rpy[2]:+.1f})  reproj={tray.reproj_px:.2f}px",
            f"FLU: F=+X L=+Y U=+Z  black={cfg.black_side_m * 1000:.2f}mm",
            f"holes={cfg.hole_rows}x{cfg.hole_cols} pitch={cfg.hole_pitch_m * 1000:.0f}mm d={cfg.hole_diameter_m * 1000:.0f}mm",
        ],
        (8, 34),
        (0, 255, 255),
    )
    return vis


def list_realsense_devices() -> list[RealSenseDevice]:
    if rs is None:
        raise RuntimeError("需要 pyrealsense2: pip install pyrealsense2")
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


def select_realsense_device(serial: str = "", camera_index: int | None = None) -> RealSenseDevice:
    devices = list_realsense_devices()
    if not devices:
        raise RuntimeError("未检测到 RealSense 相机，请检查 USB 连接")
    if serial:
        for d in devices:
            if d.serial == serial:
                return d
        raise RuntimeError(f"未找到序列号 {serial}，当前: {[d.serial for d in devices]}")
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
    def __init__(self, width: int, height: int, fps: int, device: RealSenseDevice) -> None:
        if rs is None:
            raise RuntimeError("需要 pyrealsense2: pip install pyrealsense2")
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


def print_startup(cfg: AppConfig, device: RealSenseDevice) -> None:
    print("ArUco RealSense 料盘 6D 位姿（5 码联合）")
    print(f"  相机: {device.label}  serial={device.serial}")
    print(f"  字典: {cfg.dictionary}")
    print(f"  打印(含白边): {cfg.print_size_m * 1000:.1f} mm  黑框: {cfg.black_side_m * 1000:.2f} mm")
    print(f"  四角中心距: {cfg.pitch_xy_m * 1000:.1f} mm")
    print(
        "  槽位: "
        + ", ".join(f"{slot}={cfg.slot_to_id[slot]}" for slot in SLOT_KEYS)
    )
    print("  坐标系: 原点=中心码  +X前 +Y左 +Z上")
    print(
        f"  孔位: {cfg.hole_rows}x{cfg.hole_cols}  间距 {cfg.hole_pitch_m * 1000:.1f} mm  "
        f"直径 {cfg.hole_diameter_m * 1000:.1f} mm  共 {cfg.hole_rows * cfg.hole_cols} 个"
    )
    print(f"  PnP: {cfg.pnp_flag}  refine={cfg.pnp_refine}  min_markers={cfg.min_markers}")
    print(f"  分辨率: {cfg.cam_width}x{cfg.cam_height}@{cfg.cam_fps}")
    print("  q/ESC 退出 | s 保存截图")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="board_yf100_aruco: RealSense YF-1001 太阳轮 5 码 6D 位姿")
    p.add_argument("--config", type=Path, default=HERE / "config.yaml")
    p.add_argument("--serial", type=str, default=None)
    p.add_argument("--camera-index", type=int, default=None)
    p.add_argument("--list-cameras", action="store_true")
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

    cfg_path = args.config.expanduser().resolve()
    if not cfg_path.is_file():
        raise FileNotFoundError(f"配置文件不存在: {cfg_path}")
    cfg = load_config(cfg_path)
    serial = args.serial if args.serial is not None else cfg.cam_serial
    device = select_realsense_device(serial=serial, camera_index=args.camera_index)

    detector = make_detector(get_aruco_dictionary(cfg.dictionary))
    cfg.save_dir.mkdir(parents=True, exist_ok=True)
    print_startup(cfg, device)

    cam = RealSenseRgb(cfg.cam_width, cfg.cam_height, cfg.cam_fps, device)
    cv2.namedWindow(cfg.window_name, cv2.WINDOW_NORMAL)
    frame_i = 0
    tray_ids = set(cfg.slot_to_id.values())

    try:
        while True:
            bgr = cam.read()
            if bgr is None:
                continue
            frame_i += 1

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            tray = estimate_tray_pose(corners, ids, cfg, cam.K, cam.dist)

            vis = bgr.copy()
            draw_detected_markers_thin(vis, corners, ids, tray_ids)
            overlay_tray(vis, tray, cfg, cam.K, cam.dist)

            if cfg.print_every > 0 and frame_i % cfg.print_every == 0 and tray is not None:
                rpy, t_mm, dist_mm = pose_to_rpy_tmm(tray.rvec, tray.tvec)
                print(
                    f"n={len(tray.used_ids)}/5 ids={tray.used_ids} "
                    f"t_mm=({t_mm[0]:+.1f},{t_mm[1]:+.1f},{t_mm[2]:+.1f}) "
                    f"rpy_deg=({rpy[0]:+.1f},{rpy[1]:+.1f},{rpy[2]:+.1f}) "
                    f"dist={dist_mm:.1f}mm reproj={tray.reproj_px:.2f}px"
                )

            n_tray = 0 if ids is None else sum(int(i) in tray_ids for i in ids.flatten())
            status = (
                f"tray={'ok' if tray else '---'} det={n_tray}/5 "
                f"| cam=...{cam.device.serial[-6:]}"
            )
            cv2.putText(
                vis, status, (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA,
            )
            cv2.imshow(cfg.window_name, vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                out = cfg.save_dir / f"board_yf100_aruco_{ts}.png"
                cv2.imwrite(str(out), vis)
                print(f"已保存: {out}")
    finally:
        cam.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
