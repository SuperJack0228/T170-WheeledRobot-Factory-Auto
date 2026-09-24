#!/usr/bin/env python3
"""头部相机系到 base 系的变换计算。

坐标系约定：前-左-上 (FLU, x/y/z)。
旋转约定：与 ti5_calib_software 一致，R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。

变换链：
    T_base_cam = T_base_head @ T_head_cam
    T_base_head = Trans(0, 0, link_length) @ R(roll, pitch, yaw)

其中 T_head_cam 为标定得到的「相机系 -> 头部电机末端系」4x4 矩阵（平移 mm）。
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_DIR = PACKAGE_DIR / "config"
DEFAULT_HEAD_PARAMS_YAML = DEFAULT_CONFIG_DIR / "head_params.yaml"
DEFAULT_CAMERA_TO_HEAD_YAML = DEFAULT_CONFIG_DIR / "camera_to_head_connector_result.yaml"


@dataclass
class TransformResult:
    """相机系 -> base 系变换结果。"""

    matrix: np.ndarray
    translation_xyz_mm: np.ndarray
    euler_angles_xyz_deg: np.ndarray

    def to_dict(self) -> dict:
        return {
            "matrix": self.matrix.tolist(),
            "translation_xyz_mm": self.translation_xyz_mm.tolist(),
            "euler_angles_xyz_deg": self.euler_angles_xyz_deg.tolist(),
        }


def euler_to_rotation_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """与 HandEyeCalibCommon::eulerToRotationMatrix 一致：R = Rz @ Ry @ Rx。"""
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    rx_mat = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    ry_mat = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rz_mat = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return rz_mat @ ry_mat @ rx_mat


def rotation_matrix_to_euler_xyz_deg(rotation: np.ndarray) -> np.ndarray:
    """与 HandEyeCalibCommon::rotationMatrixToEuler 一致。"""
    sy = -rotation[2, 0]
    cy = np.sqrt(max(0.0, 1.0 - sy * sy))
    if cy < 1e-8:
        rx = np.arctan2(-rotation[1, 2], rotation[1, 1])
        ry = np.arcsin(np.clip(sy, -1.0, 1.0))
        rz = 0.0
    else:
        rx = np.arctan2(rotation[2, 1] / cy, rotation[2, 2] / cy)
        ry = np.arcsin(np.clip(sy, -1.0, 1.0))
        rz = np.arctan2(rotation[1, 0] / cy, rotation[0, 0] / cy)
    return np.rad2deg([rx, ry, rz])


def make_transform(rotation: np.ndarray, translation_mm: Sequence[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation_mm, dtype=np.float64)
    return transform


def load_yaml_dict(yaml_path: str | Path) -> dict:
    if yaml is None:
        raise ImportError("读取 yaml 需要 PyYAML，请安装: pip install -r requirements.txt")

    path = Path(yaml_path)
    with path.open(encoding="utf-8") as file:
        data = yaml.safe_load(file)
    if not isinstance(data, dict):
        raise ValueError(f"{path} 内容必须是 yaml 字典")
    return data


def load_matrix_from_yaml(yaml_path: str | Path) -> np.ndarray:
    """读取标定 yaml 中的 4x4 矩阵（a. matrix）。"""
    path = Path(yaml_path)
    data = load_yaml_dict(path)

    if "a. matrix" in data:
        matrix = data["a. matrix"]
    elif isinstance(data, dict) and "matrix" in data:
        matrix = data["matrix"]
    else:
        raise ValueError(f"未在 {path} 中找到 4x4 matrix 字段")

    return np.asarray(matrix, dtype=np.float64)


class HeadCameraToBase:
    """根据头部电机角计算相机系到 base 系的变换。"""

    def __init__(
        self,
        T_head_cam: np.ndarray,
        link_length_mm: float,
    ) -> None:
        transform = np.asarray(T_head_cam, dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("T_head_cam 必须是 4x4 矩阵")
        self.T_head_cam = transform
        self.link_length_mm = float(link_length_mm)

    @classmethod
    def from_yaml(cls, calib_yaml: str | Path, link_length_mm: float) -> HeadCameraToBase:
        return cls(load_matrix_from_yaml(calib_yaml), link_length_mm)

    @classmethod
    def from_config(
        cls,
        head_params_yaml: str | Path | None = None,
        config_dir: str | Path | None = None,
    ) -> HeadCameraToBase:
        """从 config/head_params.yaml 及其中引用的标定文件初始化。"""
        params_path = Path(head_params_yaml) if head_params_yaml else DEFAULT_HEAD_PARAMS_YAML
        params = load_yaml_dict(params_path)
        base_dir = Path(config_dir) if config_dir else params_path.parent

        link_length_mm = float(params.get("link_length_mm", 162.0))
        calib_name = params.get("camera_to_head_yaml", "camera_to_head_connector_result.yaml")
        calib_path = base_dir / calib_name
        return cls.from_yaml(calib_path, link_length_mm)

    def compute(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> TransformResult:
        """输入 roll/pitch/yaw（度），输出相机系 -> base 系变换。"""
        rotation_head = euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg)
        transform_base_head = make_transform(
            rotation_head,
            (0.0, 0.0, self.link_length_mm),
        )
        transform_base_cam = transform_base_head @ self.T_head_cam
        return TransformResult(
            matrix=transform_base_cam,
            translation_xyz_mm=transform_base_cam[:3, 3].copy(),
            euler_angles_xyz_deg=rotation_matrix_to_euler_xyz_deg(transform_base_cam[:3, :3]),
        )


def format_result(result: TransformResult, precision: int = 6) -> str:
    lines = [
        "a. matrix:",
        np.array2string(result.matrix, precision=precision, suppress_small=False),
        f"b. translation_xyz_mm: {np.round(result.translation_xyz_mm, precision).tolist()}",
        f"c. euler_angles_xyz_deg: {np.round(result.euler_angles_xyz_deg, precision).tolist()}",
    ]
    return "\n".join(lines)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="根据头部 roll/pitch/yaw 计算相机系到 base 系的 4x4 变换",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=str(DEFAULT_HEAD_PARAMS_YAML),
        help="头部参数 yaml（默认 config/head_params.yaml）",
    )
    parser.add_argument(
        "--calib-yaml",
        type=str,
        default=None,
        help="可选：直接指定相机->头部末端标定 yaml，覆盖 config 中的引用",
    )
    parser.add_argument(
        "--link-length-mm",
        type=float,
        default=None,
        help="可选：直接指定连杆长度 (mm)，覆盖 config 中的值",
    )
    parser.add_argument("--roll", type=float, default=0.0, help="roll 角 (deg)")
    parser.add_argument("--pitch", type=float, default=0.0, help="pitch 角 (deg)")
    parser.add_argument("--yaw", type=float, default=0.0, help="yaw 角 (deg)")
    parser.add_argument(
        "--json",
        action="store_true",
        help="以 JSON 输出结果",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.calib_yaml is not None or args.link_length_mm is not None:
        config_path = Path(args.config)
        params = load_yaml_dict(config_path)
        calib_yaml = args.calib_yaml or str(config_path.parent / params.get(
            "camera_to_head_yaml", "camera_to_head_connector_result.yaml"
        ))
        link_length_mm = (
            args.link_length_mm
            if args.link_length_mm is not None
            else float(params.get("link_length_mm", 162.0))
        )
        calculator = HeadCameraToBase.from_yaml(calib_yaml, link_length_mm)
    else:
        calculator = HeadCameraToBase.from_config(args.config)
    result = calculator.compute(args.roll, args.pitch, args.yaw)

    if args.json:
        print(json.dumps(result.to_dict(), indent=2))
    else:
        print(format_result(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
