#!/usr/bin/env python3
"""
RealSense 在线 ArUco 检测 + 单码 6D 位姿。

基础示例：读配置 → 取 RGB/内参 → 检测码 → 按 ID 查边长 → solvePnP → 显示。
不假设码之间的相对布局；后续任务可在此之上自行扩展。

用法:
  python aruco_pose_online.py
  python aruco_pose_online.py --config config.yaml
  python aruco_pose_online.py --list-cameras
  python aruco_pose_online.py --serial <序列号>

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
    rs = None  # orch_hw 走已打开的相机，不依赖本模块开 RealSense

HERE = Path(__file__).resolve().parent
PNP_FLAGS = {
    "IPPE_SQUARE": cv2.SOLVEPNP_IPPE_SQUARE,
    "SQPNP": cv2.SOLVEPNP_SQPNP,
    "ITERATIVE": cv2.SOLVEPNP_ITERATIVE,
}


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------

@dataclass
class AppConfig:
    dictionary: str
    default_side_m: float
    id_to_side_m: dict[int, float]
    pnp_flag: str
    pnp_refine: bool
    cam_width: int
    cam_height: int
    cam_fps: int
    cam_serial: str
    axis_ratio: float
    print_every: int
    save_dir: Path
    window_name: str

    def side_length_m(self, marker_id: int) -> float:
        return self.id_to_side_m.get(marker_id, self.default_side_m)


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
    side_m: float
    rvec: np.ndarray
    tvec: np.ndarray
    corners: np.ndarray
    reproj_px: float


def load_config(path: Path) -> AppConfig:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    aruco = raw.get("aruco", {})
    pose = raw.get("pose", {})
    camera = raw.get("camera", {})
    display = raw.get("display", {})

    id_to_side: dict[int, float] = {}
    for group in aruco.get("sizes", []):
        side = float(group["side_length_m"])
        if side <= 0:
            raise ValueError(f"side_length_m 必须 > 0，得到 {side}")
        for mid in group.get("ids", []):
            mid = int(mid)
            if mid in id_to_side and abs(id_to_side[mid] - side) > 1e-9:
                raise ValueError(f"码 ID={mid} 被分配了多种尺寸")
            id_to_side[mid] = side

    save_dir = Path(display.get("save_dir", "captures"))
    if not save_dir.is_absolute():
        save_dir = path.parent / save_dir

    default_side = float(aruco.get("default_side_length_m", 0.04))
    if default_side <= 0:
        raise ValueError(f"default_side_length_m 必须 > 0，得到 {default_side}")

    return AppConfig(
        dictionary=str(aruco.get("dictionary", "DICT_5X5_1000")),
        default_side_m=default_side,
        id_to_side_m=id_to_side,
        pnp_flag=str(pose.get("pnp_flag", "IPPE_SQUARE")),
        pnp_refine=bool(pose.get("pnp_refine", True)),
        cam_width=int(camera.get("width", 1280)),
        cam_height=int(camera.get("height", 720)),
        cam_fps=int(camera.get("fps", 30)),
        cam_serial=str(camera.get("serial", "") or ""),
        axis_ratio=float(display.get("axis_length_ratio", 0.5)),
        print_every=int(display.get("print_pose_every_n_frames", 15)),
        save_dir=save_dir,
        window_name=str(display.get("window_name", "ArUco Pose")),
    )


# ---------------------------------------------------------------------------
# 底层算法
# ---------------------------------------------------------------------------

def get_aruco_dictionary(name: str) -> cv2.aruco.Dictionary:
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"未知 ArUco 字典: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_detector(dictionary: cv2.aruco.Dictionary) -> cv2.aruco.ArucoDetector:
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary, params)


def marker_corners_local(side_m: float) -> np.ndarray:
    """单码四角在码中心坐标系下的 3D 点，顺序与 OpenCV ArUco 角点一致。

    码平面 z=0，+X 向右，+Y 向上，+Z 垂直码面向外。
    """
    h = side_m * 0.5
    return np.array(
        [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
        dtype=np.float32,
    )


def rotation_matrix_to_euler_zyx_deg(R: np.ndarray) -> tuple[float, float, float]:
    """旋转矩阵 → ZYX 欧拉角（yaw, pitch, roll），单位度。"""
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


def rotation_matrix_to_xyz_rpy_deg(R: np.ndarray) -> tuple[float, float, float]:
    """与 C++ T2PosEulerAngles / rotationMatrixToEulerAngles 一致：roll, pitch, yaw（度）。"""
    sy = float(np.hypot(R[0, 0], R[1, 0]))
    if sy > 1e-6:
        roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
        yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    else:
        roll = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
        yaw = 0.0
    return roll, pitch, yaw


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


def estimate_marker_pose(
    corners: np.ndarray,
    side_m: float,
    K: np.ndarray,
    dist: np.ndarray,
    pnp_flag: str,
    pnp_refine: bool,
) -> tuple[np.ndarray, np.ndarray, float] | None:
    """单码 PnP：像素四角 + 物理边长 + 相机内参 → rvec, tvec, 重投影误差。"""
    obj_pts = marker_corners_local(side_m)
    img_pts = corners.reshape(4, 2).astype(np.float32)
    flag = PNP_FLAGS.get(pnp_flag, cv2.SOLVEPNP_IPPE_SQUARE)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist, flags=flag)
    if not ok:
        return None
    if pnp_refine and hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(obj_pts, img_pts, K, dist, rvec, tvec)
    err = mean_reprojection_error(obj_pts, img_pts, rvec, tvec, K, dist)
    return rvec.reshape(3), tvec.reshape(3), err


def estimate_poses(
    corners,
    ids,
    cfg: AppConfig,
    K: np.ndarray,
    dist: np.ndarray,
) -> list[MarkerPose]:
    if ids is None or len(corners) == 0:
        return []
    out: list[MarkerPose] = []
    for i, mid_raw in enumerate(ids.flatten()):
        mid = int(mid_raw)
        side_m = cfg.side_length_m(mid)
        solved = estimate_marker_pose(
            corners[i], side_m, K, dist, cfg.pnp_flag, cfg.pnp_refine,
        )
        if solved is None:
            continue
        rvec, tvec, reproj = solved
        out.append(
            MarkerPose(
                marker_id=mid,
                side_m=side_m,
                rvec=rvec,
                tvec=tvec,
                corners=corners[i],
                reproj_px=reproj,
            )
        )
    return out


# ---------------------------------------------------------------------------
# 显示
# ---------------------------------------------------------------------------

def draw_pose_text(vis: np.ndarray, lines: list[str], origin: tuple[int, int], color) -> None:
    x0, y0 = origin
    for j, line in enumerate(lines):
        cv2.putText(
            vis, line, (x0, y0 + j * 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA,
        )


def overlay_poses(
    vis: np.ndarray,
    poses: list[MarkerPose],
    K,
    dist,
    axis_ratio: float,
    cam2robot: np.ndarray | None = None,
) -> np.ndarray:
    T_base_cam = None
    if cam2robot is not None:
        T_base_cam = np.asarray(cam2robot, dtype=np.float64).reshape(4, 4)
    for p in poses:
        cv2.drawFrameAxes(vis, K, dist, p.rvec, p.tvec.reshape(3, 1), p.side_m * axis_ratio)
        rpy, t_mm, dist_mm = pose_to_rpy_tmm(p.rvec, p.tvec)
        lines = [
            f"ID={p.marker_id}  L={p.side_m * 1000:.0f}mm  reproj={p.reproj_px:.2f}px",
        ]
        if T_base_cam is not None:
            R, _ = cv2.Rodrigues(p.rvec)
            T_cam = np.eye(4, dtype=np.float64)
            T_cam[:3, :3] = R
            T_cam[:3, 3] = p.tvec.reshape(3)
            T_base = T_base_cam @ T_cam
            bx, by, bz = T_base[0, 3], T_base[1, 3], T_base[2, 3]
            br, bp, bw = rotation_matrix_to_xyz_rpy_deg(T_base[:3, :3])
            lines.append(f"base(m)=({bx:+.3f},{by:+.3f},{bz:+.3f})")
            lines.append(f"base rpy=({br:+.1f},{bp:+.1f},{bw:+.1f})")
        lines.append(
            f"cam(mm)=({t_mm[0]:+.0f},{t_mm[1]:+.0f},{t_mm[2]:+.0f}) dist={dist_mm:.0f}"
        )
        lines.append(f"cam rpy=({rpy[0]:+.1f},{rpy[1]:+.1f},{rpy[2]:+.1f})")
        c = p.corners.reshape(-1, 2).astype(int)
        x0 = int(np.clip(c[:, 0].min(), 0, vis.shape[1] - 1))
        block_h = len(lines) * 22
        y0 = int(np.clip(c[:, 1].min() - block_h, 16, vis.shape[0] - 8))
        draw_pose_text(vis, lines, (x0, y0), (0, 255, 255))
    return vis


# ---------------------------------------------------------------------------
# RealSense
# ---------------------------------------------------------------------------

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
    """只开彩色流，并从 profile 读取针孔内参 K / dist。"""

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


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def print_startup(cfg: AppConfig, device: RealSenseDevice) -> None:
    print("ArUco RealSense 在线 6D 位姿（单码）")
    print(f"  相机: {device.label}  serial={device.serial}")
    print(f"  字典: {cfg.dictionary}")
    print(f"  默认边长: {cfg.default_side_m * 1000:.1f} mm")
    if cfg.id_to_side_m:
        groups: dict[float, list[int]] = {}
        for mid, side in cfg.id_to_side_m.items():
            groups.setdefault(side, []).append(mid)
        for side in sorted(groups):
            print(f"  {side * 1000:.1f} mm: IDs {sorted(groups[side])}")
    else:
        print("  未配置尺寸分组，全部使用默认边长")
    print(f"  PnP: {cfg.pnp_flag}  refine={cfg.pnp_refine}")
    print(f"  分辨率: {cfg.cam_width}x{cfg.cam_height}@{cfg.cam_fps}")
    print("  q/ESC 退出 | s 保存截图")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RealSense 在线 ArUco 单码 6D 位姿")
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

    try:
        while True:
            bgr = cam.read()
            if bgr is None:
                continue
            frame_i += 1

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            poses = estimate_poses(corners, ids, cfg, cam.K, cam.dist)

            vis = bgr.copy()
            if ids is not None and len(corners) > 0:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            overlay_poses(vis, poses, cam.K, cam.dist, cfg.axis_ratio)

            if cfg.print_every > 0 and frame_i % cfg.print_every == 0:
                for p in poses:
                    rpy, t_mm, dist_mm = pose_to_rpy_tmm(p.rvec, p.tvec)
                    print(
                        f"ID={p.marker_id} L={p.side_m * 1000:.0f}mm "
                        f"t_mm=({t_mm[0]:+.1f},{t_mm[1]:+.1f},{t_mm[2]:+.1f}) "
                        f"rpy_deg=({rpy[0]:+.1f},{rpy[1]:+.1f},{rpy[2]:+.1f}) "
                        f"dist={dist_mm:.1f}mm reproj={p.reproj_px:.2f}px"
                    )

            status = f"det={len(poses)} | cam=...{cam.device.serial[-6:]}"
            cv2.putText(vis, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow(cfg.window_name, vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                out = cfg.save_dir / f"aruco_{ts}.png"
                cv2.imwrite(str(out), vis)
                print(f"已保存: {out}")
    finally:
        cam.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
