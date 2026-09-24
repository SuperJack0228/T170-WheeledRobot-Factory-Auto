#!/usr/bin/env python3
"""
RealSense 在线传送带 6D 位姿：每条皮带横贴 2 个 6×6 ArUco，独立 PnP。

皮带 0：ID0 左、ID1 右；皮带 1：ID2 左、ID3 右。
原点在两码心连线中点。+X 前、+Y 左、+Z 上。
码水平贴在上表面，码的「上」朝皮带前方。

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

# 码局部：+X 右、+Y 上（图案上方）、+Z 向外。
# 码的「上」朝皮带 +X（前）→ 码系到皮带系：皮带X=码Y，皮带Y=-码X。
R_MARKER_IN_BELT = np.array(
    [
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

BELT_COLORS = (
    (0, 200, 255),
    (255, 80, 255),
    (80, 255, 80),
    (255, 180, 80),
)


@dataclass
class BeltSpec:
    name: str
    left_id: int
    right_id: int

    @property
    def ids(self) -> set[int]:
        return {self.left_id, self.right_id}


@dataclass
class AppConfig:
    dictionary: str
    print_size_m: float
    bit_cells: int
    black_border_cells: int
    white_border_cells: int
    pitch_y_m: float
    edge_front_m: float
    edge_rear_m: float
    edge_left_m: float
    edge_right_m: float
    belts: list[BeltSpec]
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
    def black_side_m(self) -> float:
        black = self.bit_cells + 2 * self.black_border_cells
        total = black + 2 * self.white_border_cells
        return self.print_size_m * black / total

    @property
    def all_ids(self) -> set[int]:
        out: set[int] = set()
        for b in self.belts:
            out |= b.ids
        return out

    def id_to_belt(self) -> dict[int, BeltSpec]:
        mapping: dict[int, BeltSpec] = {}
        for b in self.belts:
            mapping[b.left_id] = b
            mapping[b.right_id] = b
        return mapping

    def marker_center_belt(self, belt: BeltSpec, marker_id: int) -> np.ndarray:
        half = self.pitch_y_m * 0.5
        if marker_id == belt.left_id:
            return np.array([0.0, half, 0.0], dtype=np.float64)
        if marker_id == belt.right_id:
            return np.array([0.0, -half, 0.0], dtype=np.float64)
        raise KeyError(f"ID {marker_id} 不属于 {belt.name}")

    def outline_pts_belt(self) -> np.ndarray:
        xf, xr = self.edge_front_m, -self.edge_rear_m
        yl, yr = self.edge_left_m, -self.edge_right_m
        return np.array(
            [
                [xf, yl, 0.0],
                [xf, yr, 0.0],
                [xr, yr, 0.0],
                [xr, yl, 0.0],
            ],
            dtype=np.float64,
        )


@dataclass
class RealSenseDevice:
    serial: str
    name: str

    @property
    def label(self) -> str:
        short = self.serial if len(self.serial) <= 14 else f"{self.serial[:10]}..."
        return f"{self.name} [{short}]"


@dataclass
class BeltPose:
    name: str
    rvec: np.ndarray
    tvec: np.ndarray
    reproj_px: float
    used_ids: list[int]


def load_config(path: Path) -> AppConfig:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    aruco = raw.get("aruco", {})
    belt = raw.get("belt", {})
    pose = raw.get("pose", {})
    camera = raw.get("camera", {})
    display = raw.get("display", {})

    items_raw = belt.get("items", [])
    if not items_raw:
        raise ValueError("belt.items 不能为空")
    belts: list[BeltSpec] = []
    seen: set[int] = set()
    for item in items_raw:
        spec = BeltSpec(
            name=str(item["name"]),
            left_id=int(item["left_id"]),
            right_id=int(item["right_id"]),
        )
        if spec.left_id == spec.right_id:
            raise ValueError(f"{spec.name} 左右 ID 不能相同")
        overlap = spec.ids & seen
        if overlap:
            raise ValueError(f"ID 重复: {sorted(overlap)}")
        seen |= spec.ids
        belts.append(spec)

    print_size = float(aruco.get("print_size_m", 0.03))
    bit_cells = int(aruco.get("bit_cells", 6))
    black_border = int(aruco.get("black_border_cells", 1))
    white_border = int(aruco.get("white_border_cells", 1))
    pitch_y = float(belt.get("pitch_y_m", 0.109))
    edge = belt.get("edge", {})
    edge_front = float(edge.get("front_m", 0.152))
    edge_rear = float(edge.get("rear_m", 0.152))
    edge_left = float(edge.get("left_m", 0.167))
    edge_right = float(edge.get("right_m", 0.070))
    if print_size <= 0 or pitch_y <= 0:
        raise ValueError("print_size_m / pitch_y_m 必须 > 0")
    if bit_cells <= 0 or black_border < 0 or white_border < 0:
        raise ValueError("格数配置无效")
    if min(edge_front, edge_rear, edge_left, edge_right) <= 0:
        raise ValueError("belt.edge 各边必须 > 0")

    save_dir = Path(display.get("save_dir", "captures"))
    if not save_dir.is_absolute():
        save_dir = path.parent / save_dir

    min_markers = int(pose.get("min_markers", 1))
    if min_markers < 1:
        min_markers = 1

    return AppConfig(
        dictionary=str(aruco.get("dictionary", "DICT_6X6_1000")),
        print_size_m=print_size,
        bit_cells=bit_cells,
        black_border_cells=black_border,
        white_border_cells=white_border,
        pitch_y_m=pitch_y,
        edge_front_m=edge_front,
        edge_rear_m=edge_rear,
        edge_left_m=edge_left,
        edge_right_m=edge_right,
        belts=belts,
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
        window_name=str(display.get("window_name", "board_belt_aruco")),
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


def marker_corners_in_belt(cfg: AppConfig, belt: BeltSpec, marker_id: int) -> np.ndarray:
    local = marker_corners_local(cfg.black_side_m)
    center = cfg.marker_center_belt(belt, marker_id)
    return local @ R_MARKER_IN_BELT.T + center


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


def estimate_belt_pose(
    corners,
    ids,
    cfg: AppConfig,
    belt: BeltSpec,
    K: np.ndarray,
    dist: np.ndarray,
) -> BeltPose | None:
    if ids is None or len(corners) == 0:
        return None
    obj_list: list[np.ndarray] = []
    img_list: list[np.ndarray] = []
    used: list[int] = []
    for i, mid_raw in enumerate(ids.flatten()):
        mid = int(mid_raw)
        if mid not in belt.ids:
            continue
        obj_list.append(marker_corners_in_belt(cfg, belt, mid))
        img_list.append(corners[i].reshape(4, 2).astype(np.float64))
        used.append(mid)
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
    return BeltPose(
        name=belt.name,
        rvec=rvec.reshape(3),
        tvec=tvec.reshape(3),
        reproj_px=err,
        used_ids=sorted(used),
    )


def draw_pose_text(vis: np.ndarray, lines: list[str], origin: tuple[int, int], color) -> None:
    x0, y0 = origin
    for j, line in enumerate(lines):
        cv2.putText(
            vis, line, (x0, y0 + j * 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA,
        )


def draw_detected_markers_thin(vis: np.ndarray, corners, ids, known_ids: set[int]) -> None:
    if ids is None or len(corners) == 0:
        return
    for i, mid in enumerate(ids.flatten()):
        mid = int(mid)
        pts = corners[i].reshape(-1, 2).astype(np.int32)
        color = (0, 255, 0) if mid in known_ids else (80, 80, 80)
        cv2.polylines(vis, [pts], True, color, 1, cv2.LINE_AA)
        cv2.circle(vis, (int(pts[0, 0]), int(pts[0, 1])), 2, (0, 0, 255), -1, cv2.LINE_AA)
        tx = int(pts[:, 0].min())
        ty = int(max(pts[:, 1].min() - 3, 10))
        cv2.putText(
            vis, str(mid), (tx, ty),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA,
        )


def _project_pts(
    pts: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, K, dist
) -> np.ndarray:
    proj, _ = cv2.projectPoints(pts.astype(np.float32), rvec, tvec, K, dist)
    return proj.reshape(-1, 2)


def _in_front_of_camera(pts: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec)
    cam = pts @ R.T + tvec.reshape(1, 3)
    return cam[:, 2] > 1e-4


def overlay_belt(
    vis: np.ndarray,
    pose: BeltPose,
    belt: BeltSpec,
    cfg: AppConfig,
    K,
    dist,
    color,
    text_origin: tuple[int, int],
) -> None:
    rvec = pose.rvec.reshape(3, 1)
    tvec = pose.tvec.reshape(3, 1)
    cv2.drawFrameAxes(vis, K, dist, rvec, tvec, cfg.axis_length_m, 1)

    outline = cfg.outline_pts_belt()
    if np.all(_in_front_of_camera(outline, rvec, tvec)):
        poly = _project_pts(outline, rvec, tvec, K, dist)
        if np.all(np.isfinite(poly)):
            cv2.polylines(vis, [np.round(poly).astype(np.int32)], True, color, 1, cv2.LINE_AA)

    pair = np.stack(
        [
            cfg.marker_center_belt(belt, belt.left_id),
            cfg.marker_center_belt(belt, belt.right_id),
        ],
        axis=0,
    )
    if np.all(_in_front_of_camera(pair, rvec, tvec)):
        pix = _project_pts(pair, rvec, tvec, K, dist)
        if np.all(np.isfinite(pix)):
            p0, p1 = np.round(pix).astype(int)
            cv2.line(vis, tuple(p0), tuple(p1), color, 1, cv2.LINE_AA)

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

    rpy, t_mm, dist_mm = pose_to_rpy_tmm(pose.rvec, pose.tvec)
    draw_pose_text(
        vis,
        [
            f"{pose.name} n={len(pose.used_ids)}/2 ids={pose.used_ids}",
            f"t(mm)=({t_mm[0]:+.0f},{t_mm[1]:+.0f},{t_mm[2]:+.0f}) dist={dist_mm:.0f}",
            f"rpy=({rpy[0]:+.1f},{rpy[1]:+.1f},{rpy[2]:+.1f})  reproj={pose.reproj_px:.2f}px",
        ],
        text_origin,
        color,
    )


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
    print("ArUco RealSense 传送带 6D 位姿（每条 2 码）")
    print(f"  相机: {device.label}  serial={device.serial}")
    print(f"  字典: {cfg.dictionary}")
    print(f"  打印(含白边): {cfg.print_size_m * 1000:.1f} mm  黑框: {cfg.black_side_m * 1000:.2f} mm")
    print(f"  两码心距(左右): {cfg.pitch_y_m * 1000:.1f} mm")
    print(
        f"  边缘: 前+{cfg.edge_front_m * 1000:.0f} 后-{cfg.edge_rear_m * 1000:.0f} "
        f"左+{cfg.edge_left_m * 1000:.0f} 右-{cfg.edge_right_m * 1000:.0f} mm"
    )
    for b in cfg.belts:
        print(f"  {b.name}: 左 ID{b.left_id}  右 ID{b.right_id}")
    print("  坐标系: 原点=两码中点  +X前 +Y左 +Z上")
    print(f"  PnP: {cfg.pnp_flag}  refine={cfg.pnp_refine}  min_markers={cfg.min_markers}")
    print(f"  分辨率: {cfg.cam_width}x{cfg.cam_height}@{cfg.cam_fps}")
    print("  q/ESC 退出 | s 保存截图")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="board_belt_aruco: RealSense 传送带 2 码 6D 位姿")
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
    known_ids = cfg.all_ids

    try:
        while True:
            bgr = cam.read()
            if bgr is None:
                continue
            frame_i += 1

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            poses: list[tuple[BeltSpec, BeltPose, tuple]] = []
            for i, belt in enumerate(cfg.belts):
                pose = estimate_belt_pose(corners, ids, cfg, belt, cam.K, cam.dist)
                if pose is None:
                    continue
                color = BELT_COLORS[i % len(BELT_COLORS)]
                poses.append((belt, pose, color))

            vis = bgr.copy()
            draw_detected_markers_thin(vis, corners, ids, known_ids)
            for i, (belt, pose, color) in enumerate(poses):
                overlay_belt(vis, pose, belt, cfg, cam.K, cam.dist, color, (8, 34 + i * 48))

            if cfg.print_every > 0 and frame_i % cfg.print_every == 0:
                for _, pose, _ in poses:
                    rpy, t_mm, dist_mm = pose_to_rpy_tmm(pose.rvec, pose.tvec)
                    print(
                        f"{pose.name} n={len(pose.used_ids)}/2 ids={pose.used_ids} "
                        f"t_mm=({t_mm[0]:+.1f},{t_mm[1]:+.1f},{t_mm[2]:+.1f}) "
                        f"rpy_deg=({rpy[0]:+.1f},{rpy[1]:+.1f},{rpy[2]:+.1f}) "
                        f"dist={dist_mm:.1f}mm reproj={pose.reproj_px:.2f}px"
                    )

            bits = []
            pose_by_name = {p.name: p for _, p, _ in poses}
            for belt in cfg.belts:
                bits.append(f"{belt.name}={'ok' if belt.name in pose_by_name else '---'}")
            n_det = 0 if ids is None else sum(int(i) in known_ids for i in ids.flatten())
            status = f"{' '.join(bits)} det={n_det}/{len(known_ids)} | cam=...{cam.device.serial[-6:]}"
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
                out = cfg.save_dir / f"board_belt_aruco_{ts}.png"
                cv2.imwrite(str(out), vis)
                print(f"已保存: {out}")
    finally:
        cam.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
