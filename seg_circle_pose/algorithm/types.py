"""算法输入/输出与 ID 定义。工程内空间量统一为米 (m)。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from algorithm.pose_pnp import EllipseFit

# 算法类型 ID（与业务约定）
ALGO_PNP = 0          # 分割 + 圆盘 PnP 6D
ALGO_CENTROID = 1     # 分割 + mask 深度重心，R=I

ALGO_IDS = (ALGO_PNP, ALGO_CENTROID)


@dataclass
class AlgorithmInput:
    """单帧算法输入（相机无关）。"""

    rgb: np.ndarray
    """BGR uint8, shape (H, W, 3)."""
    depth: np.ndarray | None
    """与 rgb 同分辨率的深度，单位米；无效像素为 nan。PnP 模式可为 None。"""
    K: np.ndarray
    """3×3 内参。"""
    dist_coeffs: np.ndarray
    """畸变系数，通常 5 维 (k1,k2,p1,p2,k3)。"""
    algorithm_id: int | None = None
    """None 时从 pose_params.yaml 读取（全局 default_id 或按类别 algorithm_id）。"""


@dataclass
class TargetPose:
    """单个目标输出。"""

    pose_4x4: np.ndarray
    """相机坐标系下 4×4 位姿 (R|t)，单位米。"""
    class_id: int
    confidence: float
    class_name: str = ""
    """配置中的类别名，如 feeding_cylindrical_parts。"""
    radius_m: float = 0.0
    """该目标使用的圆盘半径 (m)，来自配置 classes。"""
    algorithm_id: int = ALGO_PNP
    """该目标实际使用的位姿算法 ID。"""
    success: bool = True
    message: str = ""
    mask: np.ndarray | None = None
    """(H,W) uint8 分割 mask，供可视化。"""
    ellipse: "EllipseFit | None" = None
    """算法 0 的拟合椭圆，供可视化。"""


@dataclass
class AlgorithmOutput:
    targets: list[TargetPose] = field(default_factory=list)
