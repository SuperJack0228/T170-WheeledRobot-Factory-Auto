#!/usr/bin/env python3
"""
feeding_cylindrical_parts_alg 演示入口

参数统一由 config/pose_params.yaml 管理（分割、按类别半径、算法 0/1）。
支持多类别圆盘目标，每类独立配置 radius_m / algorithm_id。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from algorithm.config_loader import DEFAULT_CONFIG_PATH, load_pose_params
from algorithm.engine import CirclePoseEngine
from algorithm.types import ALGO_CENTROID, ALGO_PNP, AlgorithmInput
from camera.realsense_d435 import RealSenseD435
from visualization import draw_results

import cv2


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="feeding_cylindrical_parts_alg: D435 + multi-class circle pose")
    p.add_argument(
        "--config",
        type=str,
        default=str(DEFAULT_CONFIG_PATH),
        help="统一参数 YAML（默认 config/pose_params.yaml）",
    )
    p.add_argument(
        "--algorithm-id",
        type=int,
        choices=[0, 1],
        default=None,
        help="覆盖配置文件 algorithm.default_id；默认读配置(0)",
    )
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--save-dir", type=Path, default=None, help="按 s 保存 json + 可视化图")
    p.add_argument("--show-xy-axes", action="store_true", help="算法0：绘制 X/Y/Z 轴")
    return p.parse_args()


def _needs_depth(params, frame_algo: int | None) -> bool:
    if frame_algo == ALGO_CENTROID:
        return True
    if frame_algo is not None:
        return False
    if params.default_algorithm_id == ALGO_CENTROID:
        return True
    return any(s.algorithm_id == ALGO_CENTROID for s in params.classes_by_id.values())


def main() -> None:
    args = parse_args()
    params = load_pose_params(args.config)
    frame_algo = args.algorithm_id if args.algorithm_id is not None else None
    effective_algo = args.algorithm_id if args.algorithm_id is not None else params.default_algorithm_id

    engine = CirclePoseEngine(params)
    cam = RealSenseD435(
        width=args.width,
        height=args.height,
        fps=args.fps,
        enable_depth=_needs_depth(params, frame_algo),
    )

    algo_name = "PnP" if effective_algo == ALGO_PNP else "centroid"
    print(f"配置: {params.config_path}")
    print(f"模型: {params.segmentation.model_path}")
    print(
        f"YOLO: imgsz={params.segmentation.imgsz} retina_masks={params.segmentation.retina_masks}"
    )
    print(f"默认算法 ID: {params.default_algorithm_id}")
    print(
        "类别:",
        {
            s.class_id: f"{s.name} r={s.radius_m}m algo={s.algorithm_id}"
            for s in params.classes_by_id.values()
        },
    )
    print(f"本帧算法: {effective_algo} ({algo_name}) | q 退出 | s 保存 | x 切换 XYZ 轴(算法0)")
    print(f"K=\n{cam.K}")

    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)

    frame_idx = 0
    try:
        while True:
            cf = cam.grab()
            if cf is None:
                continue

            out = engine.run(
                AlgorithmInput(
                    rgb=cf.rgb,
                    depth=cf.depth,
                    K=cf.K,
                    dist_coeffs=cf.dist_coeffs,
                    algorithm_id=frame_algo,
                )
            )

            for i, t in enumerate(out.targets):
                status = "OK" if t.success else f"FAIL({t.message})"
                pos = t.pose_4x4[:3, 3]
                name = t.class_name or f"cls{t.class_id}"
                print(
                    f"[{i}] {name} algo={t.algorithm_id} conf={t.confidence:.3f} "
                    f"r={t.radius_m:.4f}m {status} "
                    f"pos_m=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f})"
                )

            vis = draw_results(
                cf.rgb,
                out,
                cf.K,
                cf.dist_coeffs,
                show_xy_axes=args.show_xy_axes,
            )
            cv2.imshow("feeding_cylindrical_parts_alg", vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("x"):
                args.show_xy_axes = not args.show_xy_axes
            if key == ord("s") and args.save_dir:
                rec = {
                    "units": "m",
                    "config": str(params.config_path),
                    "algorithm_id": effective_algo,
                    "targets": [
                        {
                            "class_id": t.class_id,
                            "class_name": t.class_name,
                            "algorithm_id": t.algorithm_id,
                            "radius_m": t.radius_m,
                            "confidence": t.confidence,
                            "success": t.success,
                            "message": t.message,
                            "pose_4x4": t.pose_4x4.tolist(),
                        }
                        for t in out.targets
                    ],
                }
                stem = time.strftime("%Y%m%d_%H%M%S") + f"_{frame_idx:06d}"
                json_path = args.save_dir / f"{stem}.json"
                img_path = args.save_dir / f"{stem}_vis.jpg"
                json_path.write_text(json.dumps(rec, indent=2), encoding="utf-8")
                cv2.imwrite(str(img_path), vis)
                print(f"saved {json_path} and {img_path}")

            frame_idx += 1
    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
