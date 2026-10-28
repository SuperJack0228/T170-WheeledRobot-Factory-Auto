#!/usr/bin/env python3
"""Read-only resource checks; does not connect to cameras/CAN or launch the robot."""
import argparse
import hashlib
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    try:
        import yaml
    except ImportError:
        print("缺少PyYAML，请使用机器人视觉模型所在的Python环境运行本工具。", file=sys.stderr)
        return 2
    root = args.root.resolve()
    config = yaml.safe_load((root / "config/move_box_params.yaml").read_text())
    vision_root = root / "feeding_cylindrical_parts_alg"
    vision = yaml.safe_load((vision_root / "config/pose_params.yaml").read_text())
    errors = []
    if not config.get("cameras", {}).get("hand_descent_correct"):
        errors.append("cameras.hand_descent_correct 未开启")
    if config.get("head_grasp", {}).get("use_hand_camera"):
        errors.append("旧 use_hand_camera 必须关闭，新伺服通过 hand_descent_correct 开启")
    if config.get("head_grasp", {}).get("descend_vel_m_s", 0) <= 0:
        errors.append("下降速度必须为正数")
    resources = [root / "lib/Ti5_Arm_2204.so", root / "config/left_hand_to_base_result.yaml",
                 root / "config/right_hand_to_base_result.yaml", root / "config/move_box_params.yaml",
                 root / "config/realsense_cameras.yaml", vision_root / "config/pose_params.yaml"]
    for key in ("model", "hand_model"):
        value = vision.get("segmentation", {}).get(key)
        if not value:
            errors.append(f"segmentation.{key} 未设置")
        else:
            path = Path(value)
            resources.append(path if path.is_absolute() else vision_root / path)
    for path in resources:
        if not path.is_file():
            errors.append(f"缺少资源：{path}")
            continue
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        try:
            display = path.relative_to(root)
        except ValueError:
            display = path
        print(f"SHA256 {digest.hexdigest()} {display}")
    for key in ("left_grasp", "right_grasp", "left_place", "right_place"):
        profile = config.get("visual_servo", {}).get(key, {})
        side="left" if key.startswith("left") else "right"
        reference=profile.get("reference_px",[config.get("cameras",{}).get(f"{side}_gripper_u"),
                                              config.get("cameras",{}).get(f"{side}_gripper_v")])
        print(f"PROFILE {key}: mode={profile.get('mode', 'geometry')} "
              f"reference_px={reference} "
              f"reference_height_m={profile.get('reference_height_m', 0)} "
              f"feature_z_offset_m={profile.get('feature_z_offset_m', 0)}")
    print("geometry 是初始几何估计，不等于已经实测标定；本工具不验证实际精度或设备连接。")
    for error in errors:
        print("ERROR:", error, file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
