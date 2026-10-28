"""料盘 Fusion 层（无相机、无 ArUco 检测）。

职责：
  - 孔位几何 ↔ YOLO 检测最近邻关联
  - 多帧扶平 T_robot_tray 的剔野与平均

Detector（tray_detect_api / online）只产码与孔几何；本模块可单测。
C++ 仍只调 tray_detect_api.detect_and_annotate / fuse_and_annotate。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

CLASS_LABEL_ZH = {
    0: "立件",
    1: "空孔",
    2: "倒件",
}


@dataclass
class HoleGeom:
    """关联用的最小孔几何（相机系 / 可选基座系）。"""

    hole_id: int
    xyz_cam: np.ndarray
    xyz_robot: np.ndarray | None = None


def associate_yolo_to_holes(
    holes: Sequence[HoleGeom],
    detections: Sequence[dict[str, Any]],
    associate_max_m: float = 0.03,
) -> list[dict[str, Any]]:
    """贪心最近邻：每个孔至多挂一个 YOLO，平面距离 ≤ associate_max_m。

    detections 项需含 class_id / class_name / conf / xyz_cam，可选 xyz_robot。
    返回与 holes 等长的列表：class_id, class_name, conf, dxy（未关联时 class_id=-1, dxy=-1）。
    """
    dets: list[dict[str, Any]] = []
    for raw in detections or []:
        xyz = np.asarray(raw.get("xyz_cam", [np.nan, np.nan, np.nan]), dtype=np.float64)
        if xyz.shape[0] < 3 or not np.all(np.isfinite(xyz[:3])):
            continue
        xyz_r = raw.get("xyz_robot")
        if xyz_r is not None:
            xyz_r = np.asarray(xyz_r, dtype=np.float64).reshape(3)
            if not np.all(np.isfinite(xyz_r)):
                xyz_r = None
        dets.append(
            {
                "class_id": int(raw.get("class_id", -1)),
                "class_name": str(raw.get("class_name", "")),
                "conf": float(raw.get("conf", 0.0)),
                "xyz_cam": xyz[:3].astype(np.float64),
                "xyz_robot": xyz_r,
            }
        )

    out: list[dict[str, Any]] = []
    claimed: set[int] = set()
    for hole in holes:
        best = None
        best_d = 1e9
        use_robot = hole.xyz_robot is not None
        if use_robot:
            hx, hy = float(hole.xyz_robot[0]), float(hole.xyz_robot[1])
        else:
            hx, hy = float(hole.xyz_cam[0]), float(hole.xyz_cam[1])
        for di, det in enumerate(dets):
            if di in claimed:
                continue
            if use_robot and det["xyz_robot"] is not None:
                src = det["xyz_robot"]
            else:
                src = det["xyz_cam"]
            dxy = float(np.hypot(float(src[0]) - hx, float(src[1]) - hy))
            if dxy < best_d:
                best_d = dxy
                best = (di, det)
        cls_id = -1
        cls_name = ""
        conf = 0.0
        dxy = -1.0
        if best is not None and best_d <= associate_max_m:
            claimed.add(best[0])
            cls_id = int(best[1]["class_id"])
            cls_name = str(best[1]["class_name"] or CLASS_LABEL_ZH.get(cls_id, ""))
            conf = float(best[1]["conf"])
            dxy = best_d
        out.append(
            {
                "hole_id": hole.hole_id,
                "class_id": cls_id,
                "class_name": CLASS_LABEL_ZH.get(cls_id, cls_name),
                "conf": conf,
                "dxy": dxy,
            }
        )
    return out


def mean_rotation_svd(Rs: list[np.ndarray]) -> np.ndarray:
    """Chordal mean：ΣR 的 SVD 投影回 SO(3)。"""
    M = np.zeros((3, 3), dtype=np.float64)
    for R in Rs:
        M += np.asarray(R, dtype=np.float64).reshape(3, 3)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0.0:
        U = U.copy()
        U[:, 2] *= -1.0
        R = U @ Vt
    return R


def geodesic_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    c = np.clip((np.trace(R_a.T @ R_b) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def reject_leveled_outliers(
    Rs: list[np.ndarray], ts: list[np.ndarray]
) -> tuple[list[np.ndarray], list[np.ndarray], int]:
    """丢掉相对中位数偏差过大的帧；至少留 2 帧。返回 (Rs, ts, dropped)。"""
    n = len(Rs)
    if n <= 2:
        return Rs, ts, 0
    t = np.stack(ts, axis=0)
    t_med = np.median(t, axis=0)
    d = np.linalg.norm(t - t_med, axis=1)
    mad_t = float(np.median(np.abs(d - np.median(d))))
    t_lim = max(0.010, 3.0 * 1.4826 * mad_t)

    R_ref = mean_rotation_svd(Rs)
    ang = np.array([geodesic_deg(R_ref, R) for R in Rs], dtype=np.float64)
    mad_a = float(np.median(np.abs(ang - np.median(ang))))
    a_lim = max(2.0, 3.0 * 1.4826 * mad_a)

    keep = (d <= t_lim) & (ang <= a_lim)
    n_keep = int(np.count_nonzero(keep))
    if n_keep < 2:
        return Rs, ts, 0
    dropped = n - n_keep
    if dropped:
        for i, ok in enumerate(keep):
            if not ok:
                print(
                    f"[tray] 丢掉第{i + 1}帧  Δt={d[i] * 1000:.1f}mm "
                    f"(限{t_lim * 1000:.1f})  ΔR={ang[i]:.2f}deg (限{a_lim:.1f})"
                )
    Rs_k = [R for R, ok in zip(Rs, keep) if ok]
    ts_k = [ti for ti, ok in zip(ts, keep) if ok]
    return Rs_k, ts_k, dropped


def fuse_leveled_tray_poses(
    Rs: list[np.ndarray], ts: list[np.ndarray]
) -> tuple[np.ndarray, np.ndarray, int]:
    """剔野后旋转 SVD 平均、平移均值。返回 (R_avg, t_avg, n_dropped)。"""
    if not Rs:
        raise ValueError("fuse_leveled_tray_poses: 空位姿列表")
    Rs_k, ts_k, n_drop = reject_leveled_outliers(Rs, ts)
    R_avg = mean_rotation_svd(Rs_k)
    t_avg = np.mean(np.stack(ts_k, axis=0), axis=0)
    return R_avg, t_avg, n_drop


def _self_test() -> None:
    # 关联：两孔两检，各挂最近
    holes = [
        HoleGeom(1, np.array([0.0, 0.0, 1.0]), np.array([0.1, 0.2, 0.5])),
        HoleGeom(2, np.array([0.0, 0.0, 1.0]), np.array([0.2, 0.2, 0.5])),
    ]
    dets = [
        {"class_id": 0, "conf": 0.9, "xyz_cam": [0, 0, 1], "xyz_robot": [0.105, 0.2, 0.5]},
        {"class_id": 1, "conf": 0.8, "xyz_cam": [0, 0, 1], "xyz_robot": [0.195, 0.2, 0.5]},
    ]
    assoc = associate_yolo_to_holes(holes, dets, associate_max_m=0.03)
    assert assoc[0]["class_id"] == 0 and assoc[1]["class_id"] == 1, assoc

    # 旋转平均：单位阵
    R = np.eye(3)
    R_m = mean_rotation_svd([R, R, R])
    assert np.allclose(R_m, R), R_m

    # 融合：平移取均值
    ts = [np.array([0.0, 0.0, 0.5]), np.array([0.02, 0.0, 0.5]), np.array([0.01, 0.0, 0.5])]
    Rs = [R, R, R]
    _, t_avg, n_drop = fuse_leveled_tray_poses(Rs, ts)
    assert n_drop == 0
    assert abs(t_avg[0] - 0.01) < 1e-9
    print("tray_fusion self-test OK")


if __name__ == "__main__":
    _self_test()
