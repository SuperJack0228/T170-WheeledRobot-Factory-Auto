#!/usr/bin/env python3
"""
RealSense 三相机位置标定：逐个预览画面，点击「头 / 右手 / 左手」绑定序列号，写出 YAML。

用法:
  cd market_simple_2026_1_21_using
  python config/generate_camera_config.py
  python config/generate_camera_config.py -o config/realsense_cameras.yaml

依赖: pip install pyrealsense2 opencv-python pyyaml
"""

from __future__ import annotations

import argparse
import sys
import textwrap
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
    import yaml
except ImportError as e:
    print("依赖缺失，请先安装: pip install pyrealsense2 opencv-python pyyaml", file=sys.stderr)
    raise SystemExit(1) from e

CONFIG_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT = CONFIG_DIR / "realsense_cameras.yaml"

SLOTS = (
    ("head", "头部", (80, 180, 255)),
    ("right_hand", "右手", (80, 255, 120)),
    ("left_hand", "左手", (255, 180, 80)),
)


@dataclass
class RsDevice:
    serial: str
    name: str
    usb_type: str


@dataclass
class UiState:
    hover: str | None = None
    message: str = ""
    message_color: tuple[int, int, int] = (255, 255, 255)


def list_realsense_devices() -> list[RsDevice]:
    ctx = rs.context()
    out: list[RsDevice] = []
    for dev in ctx.query_devices():
        serial = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        try:
            usb_type = dev.get_info(rs.camera_info.usb_type_descriptor)
        except Exception:
            usb_type = ""
        out.append(RsDevice(serial=serial, name=name, usb_type=usb_type))
    return out


def button_rects(w: int, h: int) -> dict[str, tuple[int, int, int, int]]:
    margin = 16
    gap = 12
    btn_h = 52
    btn_w = (w - margin * 2 - gap * 2) // 3
    y1 = h - margin - btn_h
    rects: dict[str, tuple[int, int, int, int]] = {}
    for i, (slot_id, _, _) in enumerate(SLOTS):
        x1 = margin + i * (btn_w + gap)
        rects[slot_id] = (x1, y1, x1 + btn_w, y1 + btn_h)
    return rects


def draw_ui(
    frame: np.ndarray,
    device: RsDevice,
    index: int,
    total: int,
    assigned: dict[str, str],
    state: UiState,
) -> np.ndarray:
    vis = frame.copy()
    h, w = vis.shape[:2]

    overlay = vis.copy()
    cv2.rectangle(overlay, (0, 0), (w, 96), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.65, vis, 0.35, 0, vis)

    title = f"相机 {index + 1}/{total}  |  SN: {device.serial}"
    cv2.putText(vis, title, (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
    sub = f"{device.name}  USB:{device.usb_type or '?'}"
    cv2.putText(vis, sub, (16, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (200, 200, 200), 1, cv2.LINE_AA)

    cv2.putText(
        vis,
        "请点击下方按钮绑定该相机位置，或按 H / R / L",
        (16, h - 78),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (230, 230, 230),
        1,
        cv2.LINE_AA,
    )

    rects = button_rects(w, h)
    for slot_id, label, color in SLOTS:
        x1, y1, x2, y2 = rects[slot_id]
        fill = color if state.hover == slot_id else tuple(c // 2 for c in color)
        cv2.rectangle(vis, (x1, y1), (x2, y2), fill, -1)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 255, 255), 2)

        taken = slot_id in assigned
        text = f"{label}" + (" (已绑)" if taken else "")
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.62, 2)
        tx = x1 + (x2 - x1 - tw) // 2
        ty = y1 + (y2 - y1 + th) // 2
        cv2.putText(vis, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (20, 20, 20), 2, cv2.LINE_AA)

    y = 118
    cv2.putText(vis, "已绑定:", (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 255, 180), 1, cv2.LINE_AA)
    y += 28
    for slot_id, label, _ in SLOTS:
        sn = assigned.get(slot_id, "—")
        cv2.putText(vis, f"  {label}: {sn}", (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 200, 200), 1, cv2.LINE_AA)
        y += 24

    if state.message:
        cv2.putText(vis, state.message, (16, h - 110), cv2.FONT_HERSHEY_SIMPLEX, 0.62, state.message_color, 2, cv2.LINE_AA)

    cv2.putText(vis, "N下一台  S跳过  Q退出", (w - 260, h - 110), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (180, 180, 180), 1, cv2.LINE_AA)
    return vis


def open_preview(serial: str, width: int, height: int, fps: int) -> rs.pipeline:
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipeline.start(cfg)
    for _ in range(15):
        pipeline.wait_for_frames()
    return pipeline


def save_yaml(path: Path, assigned: dict[str, str], devices: list[RsDevice]) -> None:
    by_serial = {d.serial: d for d in devices}
    cameras = {}
    for slot_id, label, _ in SLOTS:
        serial = assigned.get(slot_id)
        if not serial:
            continue
        dev = by_serial.get(serial)
        cameras[slot_id] = {
            "label": label,
            "serial": serial,
            "name": dev.name if dev else "",
            "usb_type": dev.usb_type if dev else "",
        }

    doc = {
        "generated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "camera_count": len(devices),
        "realsense": cameras,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    header = textwrap.dedent(
        """\
        # RealSense 相机位置与序列号（由 generate_camera_config.py 生成）
        # 键: head / right_hand / left_hand
        """
    )
    with path.open("w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False, default_flow_style=False)


def assign_slot(
    slot_id: str,
    device: RsDevice,
    assigned: dict[str, str],
    state: UiState,
) -> bool:
    label = next(l for sid, l, _ in SLOTS if sid == slot_id)
    if slot_id in assigned:
        state.message = f"{label} 已绑定 SN {assigned[slot_id]}"
        state.message_color = (0, 180, 255)
        return False
    assigned[slot_id] = device.serial
    state.message = f"已绑定 {label} -> {device.serial}"
    state.message_color = (80, 255, 120)
    return True


def run_wizard(
    devices: list[RsDevice],
    output: Path,
    width: int,
    height: int,
    fps: int,
) -> None:
    assigned: dict[str, str] = {}
    used_serials: set[str] = set()
    win = "realsense_camera_config"

    for idx, device in enumerate(devices):
        if device.serial in used_serials:
            continue

        state = UiState()
        click_slot: list[str | None] = [None]
        hover_slot: list[str | None] = [None]
        rects_holder: dict[str, tuple[int, int, int, int]] = {}

        def on_mouse(event, x, y, _flags, _userdata) -> None:
            hover_slot[0] = None
            for sid, (x1, y1, x2, y2) in rects_holder.items():
                if x1 <= x <= x2 and y1 <= y <= y2:
                    hover_slot[0] = sid
                    if event == cv2.EVENT_LBUTTONDOWN:
                        click_slot[0] = sid
                    break

        try:
            pipeline = open_preview(device.serial, width, height, fps)
        except Exception as e:
            print(f"无法打开 SN={device.serial}: {e}", file=sys.stderr)
            continue

        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, min(width, 1280), min(height + 40, 760))
        cv2.setMouseCallback(win, on_mouse)

        advance = False
        try:
            while not advance:
                frames = pipeline.wait_for_frames()
                color = frames.get_color_frame()
                if not color:
                    continue
                frame = np.asanyarray(color.get_data())
                state.hover = hover_slot[0]
                rects_holder.update(button_rects(frame.shape[1], frame.shape[0]))
                vis = draw_ui(frame, device, idx, len(devices), assigned, state)
                cv2.imshow(win, vis)

                if click_slot[0]:
                    sid = click_slot[0]
                    click_slot[0] = None
                    if assign_slot(sid, device, assigned, state):
                        used_serials.add(device.serial)
                        advance = True

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    raise KeyboardInterrupt
                if key in (ord("n"), ord("s")):
                    state.message = "已跳过本相机"
                    advance = True
                elif key in (ord("h"), ord("H")):
                    if assign_slot("head", device, assigned, state):
                        used_serials.add(device.serial)
                        advance = True
                elif key in (ord("r"), ord("R")):
                    if assign_slot("right_hand", device, assigned, state):
                        used_serials.add(device.serial)
                        advance = True
                elif key in (ord("l"), ord("L")):
                    if assign_slot("left_hand", device, assigned, state):
                        used_serials.add(device.serial)
                        advance = True

                if len(assigned) >= len(SLOTS):
                    advance = True
        finally:
            pipeline.stop()
            cv2.destroyWindow(win)

        if len(assigned) >= len(SLOTS):
            break

    save_yaml(output, assigned, devices)
    cv2.destroyAllWindows()

    print(f"\n已写入: {output}")
    for slot_id, label, _ in SLOTS:
        print(f"  {label:4s} ({slot_id:10s}): {assigned.get(slot_id, '(未绑定)')}")

    missing = [label for sid, label, _ in SLOTS if sid not in assigned]
    if missing:
        print(f"\n警告: 以下位置未绑定: {', '.join(missing)}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RealSense 三相机位置标定并生成 YAML")
    p.add_argument("-o", "--output", type=Path, default=DEFAULT_OUTPUT, help="输出配置文件")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    devices = list_realsense_devices()
    if not devices:
        print("未检测到 RealSense 相机，请检查 USB 连接。", file=sys.stderr)
        raise SystemExit(1)

    print(f"检测到 {len(devices)} 台 RealSense:")
    for i, d in enumerate(devices, 1):
        print(f"  [{i}] SN={d.serial}  {d.name}  USB={d.usb_type}")

    if len(devices) < len(SLOTS):
        print(f"\n提示: 期望绑定 {len(SLOTS)} 个位置，当前只检测到 {len(devices)} 台。", file=sys.stderr)

    print("\n操作说明:")
    print("  画面下方点击 [头部] [右手] [左手] 绑定当前相机，然后自动切下一台")
    print("  快捷键: H=头  R=右手  L=左手  N/S=跳过  Q=退出")
    print()

    try:
        run_wizard(devices, args.output, args.width, args.height, args.fps)
    except KeyboardInterrupt:
        print("\n已取消。")
        cv2.destroyAllWindows()
        raise SystemExit(130)


if __name__ == "__main__":
    main()
