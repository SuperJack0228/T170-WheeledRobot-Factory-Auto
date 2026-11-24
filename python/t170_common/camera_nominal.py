"""Convert declared camera frames into the existing servo image convention."""
from copy import deepcopy
import math

from .arm_model import legacy_flange_bridge, multiply

SERVO_IMAGES = {"left": "raw_rgb", "right": "rgb_rotated_180"}


def image_rotations(cameras):
    result = {}
    for side, default in [('left', 0), ('right', 180)]:
        rotation = cameras['realsense'][side+'_hand'].get('image_rotation_deg', default)
        if type(rotation) is not int or rotation not in (0, 180):
            raise ValueError('image_rotation_deg must be 0 or 180')
        result[side] = rotation
    return result


def hand_eye_for_servo(eye, binding, frames, side, image_rotation_deg=None):
    result = deepcopy(eye)
    key = "a. matrix" if "a. matrix" in result else "matrix"
    matrix = result[key]
    if (not isinstance(matrix, list) or len(matrix) != 4 or
            any(not isinstance(row, list) or len(row) != 4 for row in matrix) or
            any(not isinstance(v, (int, float)) or not math.isfinite(v) for row in matrix for v in row)):
        raise ValueError("Hand-eye must contain a finite 4x4 matrix")
    parent = result.get("parent_frame", binding["hand_eye_coordinates"])
    if parent == "urdf_wrist":
        bridge = deepcopy(frames[side]["sdk_flange_from_urdf_wrist"])
    elif parent == "legacy_t7_flange":
        bridge = legacy_flange_bridge(frames, side)
    elif parent in ("sdk_flange", "selected_sdk_flange"):
        bridge = [[float(i == j) for j in range(4)] for i in range(4)]
    else:
        raise ValueError("Unknown hand-eye parent frame: " + str(parent))
    for i in range(3):
        bridge[i][3] *= 1000  # Source hand-eye translations are millimetres.
    matrix = multiply(bridge, matrix)
    target = SERVO_IMAGES[side] if image_rotation_deg is None else ('rgb_rotated_180' if image_rotation_deg == 180 else 'raw_rgb')
    legacy = {'left':'rgb_rotated_180','right':'raw_rgb'}
    convention = result.get("image_convention", legacy[side] if parent=='legacy_t7_flange' else target)
    if convention not in ("raw_rgb", "rgb_rotated_180"):
        raise ValueError("Unknown camera image convention: " + str(convention))
    if convention != target:
        matrix = multiply(matrix, [[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    result[key] = matrix
    for name in ("b. translation_xyz_mm", "c. euler_angles_xyz_deg"):
        result.pop(name, None)
    result["parent_frame"] = "selected_sdk_flange"
    result["source_parent_frame"] = parent
    if "image_convention" in eye:
        result["source_image_convention"] = convention
        result["image_convention"] = target
    return result


def nominal_files():
    base = "config/robot_models/t170c_v2_common/cameras/"
    return {
        "config/head/geometry.yaml": base + "head_geometry.yaml",
        "config/head/extrinsics.yaml": base + "head_nominal.yaml",
        "config/head/extrinsics_urdf.yaml": base + "head_nominal.yaml",
        "config/camera_to_base_result.yaml": base + "head_fallback_nominal.yaml",
        "config/left_hand_to_base_result.yaml": base + "left_nominal.yaml",
        "config/right_hand_to_base_result.yaml": base + "right_nominal.yaml",
    }
