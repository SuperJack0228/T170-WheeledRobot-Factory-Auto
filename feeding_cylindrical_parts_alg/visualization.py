"""Compact visualization used by the embedded T170C cylinder detector."""

from __future__ import annotations

import cv2
import numpy as np

_COLORS = ((30, 220, 30), (255, 120, 20), (40, 180, 255), (220, 80, 220))


def draw_results(bgr, output, K, dist_coeffs, show_xy_axes=False):
    image = np.asarray(bgr, dtype=np.uint8).copy()
    overlay = image.copy()
    targets = list(getattr(output, "targets", []))

    for index, target in enumerate(targets):
        color = _COLORS[int(getattr(target, "class_id", index)) % len(_COLORS)]
        mask = getattr(target, "mask", None)
        if mask is not None:
            selected = np.asarray(mask) > 0
            overlay[selected] = color
    image = cv2.addWeighted(overlay, 0.28, image, 0.72, 0.0)

    for index, target in enumerate(targets):
        color = _COLORS[int(getattr(target, "class_id", index)) % len(_COLORS)]
        ellipse = getattr(target, "ellipse", None)
        if ellipse is not None:
            cv2.ellipse(image, (ellipse.center, ellipse.axes, ellipse.angle_deg), color, 2)

        ok = bool(getattr(target, "success", False))
        pose = np.asarray(getattr(target, "pose_4x4", np.eye(4)), dtype=np.float64)
        if ok and pose.shape == (4, 4):
            rvec, _ = cv2.Rodrigues(pose[:3, :3])
            tvec = pose[:3, 3].reshape(3, 1)
            axis = max(float(getattr(target, "radius_m", 0.025)), 0.01)
            cv2.drawFrameAxes(image, np.asarray(K), np.asarray(dist_coeffs), rvec, tvec,
                              axis if show_xy_axes else axis * 0.7)

        label = (f"{getattr(target, 'class_name', 'cylinder')} "
                 f"{float(getattr(target, 'confidence', 0.0)):.2f} "
                 f"algo={int(getattr(target, 'algorithm_id', -1))}")
        cv2.putText(image, label, (12, 28 + index * 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color if ok else (0, 0, 255), 2, cv2.LINE_AA)
    return image
