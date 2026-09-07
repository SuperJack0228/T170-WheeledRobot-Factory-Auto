#!/usr/bin/env python3
"""
按 config/realsense_cameras.yaml 选择位置（头/右手/左手）打开对应 RealSense 预览。

用法:
  python config/view_realsense_camera.py
  python config/view_realsense_camera.py --config config/realsense_cameras.yaml

界面用 tkinter + 中文字体，避免 OpenCV putText 显示问号。
"""

from __future__ import annotations

import argparse
import sys
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import font as tkfont
from tkinter import messagebox

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageTk

try:
    import pyrealsense2 as rs
    import yaml
except ImportError:
    print("依赖缺失: pip install pyrealsense2 opencv-python pillow pyyaml", file=sys.stderr)
    raise SystemExit(1)

CONFIG_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = CONFIG_DIR / "realsense_cameras.yaml"

ORDER = ("head", "right_hand", "left_hand")

CJK_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
)

TK_FONT_CANDIDATES = (
    "Noto Sans CJK SC",
    "WenQuanYi Micro Hei",
    "WenQuanYi Zen Hei",
    "Droid Sans Fallback",
    "Sans",
)


@dataclass
class CameraEntry:
    slot_id: str
    label: str
    serial: str
    name: str
    usb_type: str


def find_pil_font(size: int = 22) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for path in CJK_FONT_CANDIDATES:
        if Path(path).is_file():
            try:
                return ImageFont.truetype(path, size=size)
            except OSError:
                continue
    return ImageFont.load_default()


def pick_tk_font(root: tk.Tk, size: int = 15, weight: str = "normal") -> tkfont.Font:
    families = set(tkfont.families(root))
    for name in TK_FONT_CANDIDATES:
        if name in families:
            return tkfont.Font(root=root, family=name, size=size, weight=weight)
    return tkfont.Font(root=root, size=size, weight=weight)


def load_cameras(path: Path) -> dict[str, CameraEntry]:
    if not path.is_file():
        raise FileNotFoundError(f"配置文件不存在: {path}")
    with path.open(encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    raw = doc.get("realsense") or {}
    out: dict[str, CameraEntry] = {}
    for slot_id, info in raw.items():
        if not isinstance(info, dict):
            continue
        out[slot_id] = CameraEntry(
            slot_id=slot_id,
            label=str(info.get("label") or slot_id),
            serial=str(info.get("serial") or ""),
            name=str(info.get("name") or ""),
            usb_type=str(info.get("usb_type") or ""),
        )
    return out


class RealSenseViewer:
    def __init__(
        self,
        cameras: dict[str, CameraEntry],
        width: int,
        height: int,
        fps: int,
    ) -> None:
        self.cameras = cameras
        self.width = width
        self.height = height
        self.fps = fps

        self.root = tk.Tk()
        self.root.title("RealSense 相机预览")
        self.root.protocol("WM_DELETE_WINDOW", self.on_quit)

        self.btn_font = pick_tk_font(self.root, 16, "bold")
        self.hint_font = pick_tk_font(self.root, 12)
        self.pil_font_lg = find_pil_font(26)
        self.pil_font_sm = find_pil_font(18)

        self.pipeline: rs.pipeline | None = None
        self.current: CameraEntry | None = None
        self._photo: ImageTk.PhotoImage | None = None
        self._running = True

        self._build_ui()
        self.status_var.set("请点击上方按钮选择要打开的相机位置")

    def _build_ui(self) -> None:
        top = tk.Frame(self.root, padx=12, pady=10)
        top.pack(side=tk.TOP, fill=tk.X)

        tk.Label(top, text="选择相机位置", font=self.btn_font).pack(anchor=tk.W, pady=(0, 8))

        btn_row = tk.Frame(top)
        btn_row.pack(fill=tk.X)

        for slot_id in ORDER:
            cam = self.cameras.get(slot_id)
            if cam is None:
                continue
            text = f"{cam.label}\nSN {cam.serial[-6:]}"
            tk.Button(
                btn_row,
                text=text,
                font=self.btn_font,
                width=10,
                height=2,
                command=lambda s=slot_id: self.open_slot(s),
            ).pack(side=tk.LEFT, padx=6)

        tk.Button(
            btn_row,
            text="退出",
            font=self.btn_font,
            width=8,
            height=2,
            command=self.on_quit,
        ).pack(side=tk.RIGHT, padx=6)

        self.status_var = tk.StringVar()
        tk.Label(top, textvariable=self.status_var, font=self.hint_font, fg="#333").pack(
            anchor=tk.W, pady=(8, 0)
        )

        self.video_label = tk.Label(self.root, bg="#111")
        self.video_label.pack(side=tk.TOP, padx=8, pady=8)

        hint = "提示：可随时点击按钮切换相机 | 关闭窗口退出"
        tk.Label(self.root, text=hint, font=self.hint_font, fg="#666").pack(pady=(0, 10))

    def open_slot(self, slot_id: str) -> None:
        cam = self.cameras.get(slot_id)
        if cam is None:
            messagebox.showwarning("未配置", f"配置里没有位置: {slot_id}")
            return
        if not cam.serial:
            messagebox.showwarning("序列号为空", f"{cam.label} 未填写 serial")
            return

        self.stop_pipeline()
        try:
            pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(cam.serial)
            cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            pipeline.start(cfg)
            for _ in range(10):
                pipeline.wait_for_frames()
        except Exception as e:
            messagebox.showerror(
                "打开失败",
                f"无法打开 {cam.label}\n序列号: {cam.serial}\n\n{e}",
            )
            return

        self.pipeline = pipeline
        self.current = cam
        self.status_var.set(f"当前: {cam.label}  |  {cam.name}  |  SN {cam.serial}")
        self._tick()

    def stop_pipeline(self) -> None:
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None

    def _draw_overlay(self, bgr: np.ndarray, cam: CameraEntry) -> Image.Image:
        rgb = bgr[:, :, ::-1]
        img = Image.fromarray(rgb)
        draw = ImageDraw.Draw(img)

        lines = [
            f"位置: {cam.label}",
            f"型号: {cam.name}",
            f"序列号: {cam.serial}",
        ]
        bar_h = 14 + len(lines) * 30
        draw.rectangle((0, 0, img.width, bar_h), fill=(0, 0, 0))
        y = 12
        for i, line in enumerate(lines):
            fnt = self.pil_font_lg if i == 0 else self.pil_font_sm
            draw.text((14, y), line, font=fnt, fill=(255, 255, 255))
            y += 34 if i == 0 else 28

        return img

    def _tick(self) -> None:
        if not self._running:
            return
        if self.pipeline is None or self.current is None:
            self.root.after(200, self._tick)
            return

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color = frames.get_color_frame()
            if color:
                bgr = np.asanyarray(color.get_data())
                pil_img = self._draw_overlay(bgr, self.current)
                disp_w = min(pil_img.width, 1280)
                disp_h = int(pil_img.height * disp_w / pil_img.width)
                resample = getattr(Image, "Resampling", Image).BILINEAR
                pil_img = pil_img.resize((disp_w, disp_h), resample)
                self._photo = ImageTk.PhotoImage(pil_img)
                self.video_label.configure(image=self._photo)
        except Exception as e:
            self.status_var.set(f"取流失败: {e}")
            self.stop_pipeline()

        self.root.after(33, self._tick)

    def on_quit(self) -> None:
        self._running = False
        self.stop_pipeline()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="按位置打开 RealSense 相机预览")
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    try:
        cameras = load_cameras(args.config)
    except FileNotFoundError as e:
        print(e, file=sys.stderr)
        print("请先运行: python config/generate_camera_config.py", file=sys.stderr)
        raise SystemExit(1)

    if not cameras:
        print("配置文件中没有 realsense 条目", file=sys.stderr)
        raise SystemExit(1)

    print("已加载相机配置:")
    for slot_id in ORDER:
        cam = cameras.get(slot_id)
        if cam:
            print(f"  {cam.label} ({slot_id}): SN={cam.serial}")

    RealSenseViewer(cameras, args.width, args.height, args.fps).run()


if __name__ == "__main__":
    main()
