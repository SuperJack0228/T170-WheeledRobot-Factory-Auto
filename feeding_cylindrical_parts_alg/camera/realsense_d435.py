"""RealSense D435：输出 RGB、可选对齐深度 (m)、内参与畸变。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError as e:
    raise ImportError("需要 pyrealsense2: pip install pyrealsense2") from e


@dataclass
class CameraFrame:
    rgb: np.ndarray
    """BGR uint8 (H, W, 3)."""
    depth: np.ndarray | None
    """float32 米，与 rgb 同尺寸；无效为 nan。无深度时为 None。"""
    K: np.ndarray
    """3×3 内参。"""
    dist_coeffs: np.ndarray
    """畸变 (5,)."""


class RealSenseD435:
    def __init__(
        self,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        enable_depth: bool = False,
    ) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_depth = enable_depth

        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        if enable_depth:
            cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        profile = self._pipeline.start(cfg)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = np.array(
            [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]],
            dtype=np.float64,
        )
        self.dist_coeffs = np.array(list(intr.coeffs[:5]), dtype=np.float64)

        self._align = rs.align(rs.stream.color) if enable_depth else None
        self._depth_scale = 0.001
        if enable_depth:
            sensor = profile.get_device().first_depth_sensor()
            self._depth_scale = float(sensor.get_depth_scale())

    def grab(self) -> CameraFrame | None:
        frames = self._pipeline.wait_for_frames()
        if self.enable_depth and self._align is not None:
            aligned = self._align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f:
                return None
            rgb = np.asanyarray(color_f.get_data())
            depth = self._depth_to_meters(depth_f)
        else:
            color_f = frames.get_color_frame()
            if not color_f:
                return None
            rgb = np.asanyarray(color_f.get_data())
            depth = None

        return CameraFrame(rgb=rgb, depth=depth, K=self.K.copy(), dist_coeffs=self.dist_coeffs.copy())

    def _depth_to_meters(self, depth_frame) -> np.ndarray:
        raw = np.asanyarray(depth_frame.get_data())
        depth_m = raw.astype(np.float32) * self._depth_scale
        depth_m[raw == 0] = np.nan
        return depth_m

    def stop(self) -> None:
        self._pipeline.stop()
