#!/usr/bin/env python3
"""Derive T170C V2 MDH profiles from CAD URDF. Calculation only, no SDK or IO.

This converter is scoped to the two supplied T170C V2 arm chains. It verifies
complete FK equivalence before writing. It does not infer encoder zero, TCP,
physical payload ratings, collision clearance or analytic-IK compatibility.
"""
import argparse
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

SUFFIXES = ('SHOULDER_P', 'SHOULDER_R', 'SHOULDER_Y', 'ELBOW_Y', 'WRIST_P', 'WRIST_Y', 'WRIST_R')
ALPHA = np.array([1, 1, 1, -1, 1, -1, -1]) * math.pi / 2
OFFSET = np.array([1, -1, 1, 0, 0, -1, 0]) * math.pi / 2


def unit(v):
    n = np.linalg.norm(v)
    if not np.isfinite(n) or n < 1e-10:
        raise ValueError('Degenerate joint axis/common normal')
    return v / n


def rot(axis, q):
    x, y, z = unit(axis)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(q)*k + (1-np.cos(q))*(k @ k)


def origin(j):
    n = j.find('origin')
    t = np.eye(4)
    if n is not None:
        t[:3, 3] = np.fromstring(n.get('xyz', '0 0 0'), sep=' ')
        rx, ry, rz = np.fromstring(n.get('rpy', '0 0 0'), sep=' ')
        t[:3, :3] = rot(np.array([0., 0, 1]), rz) @ rot(np.array([0., 1, 0]), ry) @ rot(np.array([1., 0, 0]), rx)
    return t


def mdh(row, q):
    al, a, d, off = row
    ca, sa, c, s = np.cos(al), np.sin(al), np.cos(q+off), np.sin(q+off)
    return np.array([[c, -s, 0, a], [s*ca, c*ca, -sa, -sa*d],
                     [s*sa, c*sa, ca, ca*d], [0, 0, 0, 1.]])


def fk_mdh(rows, q):
    t = np.eye(4)
    for row, angle in zip(rows, q):
        t = t @ mdh(row, angle)
    return t


def fk_urdf(joints, q):
    t = np.eye(4)
    for j, angle in zip(joints, q):
        motion = np.eye(4)
        motion[:3, :3] = rot(np.fromstring(j.find('axis').get('xyz'), sep=' '), angle)
        t = t @ origin(j) @ motion
    return t


def clean(a):
    a = np.asarray(a).copy()
    a[np.abs(a) < 1e-14] = 0
    return a.tolist()


def derive(urdf_bytes, model_id, rated_payload_kg):
    robot = ET.fromstring(urdf_bytes)
    nodes = {j.get('name'): j for j in robot.findall('joint')}
    sdk = {'_plan': {'sample_rate': 0.005}, 'T7_170': {'robot_dof': 7, 'default': 'right'}}
    conventions, frames, evidence = {}, {}, {}
    rng = np.random.default_rng(17020261117)
    base = origin(nodes['NECK_Y'])
    for side, prefix in [('left', 'L'), ('right', 'R')]:
        joints = [nodes[prefix+'_'+s] for s in SUFFIXES]
        if joints[0].find('parent').get('link') != 'WAIST_Y_S':
            raise ValueError('Unexpected T170 arm root')
        for a, b in zip(joints, joints[1:]):
            if a.find('child').get('link') != b.find('parent').get('link'):
                raise ValueError('Broken T170 arm chain')
        direction = np.array([-1, 1, 1, -1, 1, -1, 1] if side == 'left' else [1]*7)
        offset = np.array([0, -math.pi/2 if side == 'left' else math.pi/2, 0, 0, 0, 0, 0])
        q0 = -offset / direction
        positions, axes = [], []
        t = np.linalg.inv(base)
        for j, sign in zip(joints, direction):
            t = t @ origin(j)
            positions.append(t[:3, 3].copy())
            axes.append(unit(t[:3, :3] @ np.fromstring(j.find('axis').get('xyz'), sep=' ')) * sign)
        # Preserve the established SDK frame choice wherever possible. These
        # reference orientations define coordinates, not physical link lengths.
        reference = np.array([fk_mdh(np.column_stack([ALPHA[:i+1], np.zeros((i+1, 2)), OFFSET[:i+1]]), q0[:i+1])
                              for i in range(7)])
        zero_frames = []
        for i, (p, z) in enumerate(zip(positions, axes)):
            if i < 6:
                zn = axes[i+1]
                ts = np.linalg.lstsq(np.column_stack([z, -zn]), positions[i+1]-p, rcond=None)[0]
                point, x = p + ts[0]*z, unit(np.cross(z, zn))
                if x @ reference[i, :3, 0] < 0:
                    x = -x
            else:
                point = p
                x = unit(reference[i, :3, 0] - z*(z @ reference[i, :3, 0]))
            f = np.eye(4)
            f[:3, :3] = np.column_stack([x, np.cross(z, x), z])
            f[:3, 3] = point
            zero_frames.append(f)
        previous, rows = np.eye(4), []
        for i, f in enumerate(zero_frames):
            a = np.linalg.inv(previous) @ f
            alpha = math.atan2(-a[1, 2], a[2, 2])
            theta = math.atan2(-a[0, 1], a[0, 0])
            theta += 2*math.pi*round((q0[i]+OFFSET[i]-theta)/(2*math.pi))
            row = [alpha, a[0, 3], -math.sin(alpha)*a[1, 3]+math.cos(alpha)*a[2, 3], theta-q0[i]]
            if np.max(np.abs(a-mdh(row, q0[i]))) > 1e-11:
                raise ValueError('URDF cannot be represented with the selected MDH frames')
            rows.append(row)
            previous = f
        rows = np.array(clean(rows))
        bridge = np.linalg.inv(base @ zero_frames[-1]) @ fk_urdf(joints, np.zeros(7))
        lo = np.array([float(j.find('limit').get('lower')) for j in joints])
        hi = np.array([float(j.find('limit').get('upper')) for j in joints])
        if not np.all(np.isfinite(rows)) or not np.all(lo < hi):
            raise ValueError('Invalid model parameters')
        max_error = 0.
        samples = np.vstack([np.zeros((1, 7)), lo, hi, rng.uniform(lo, hi, size=(1000, 7))])
        for u in samples:
            calculated = base @ fk_mdh(rows, (u-offset)/direction) @ bridge
            max_error = max(max_error, float(np.max(np.abs(calculated-fk_urdf(joints, u)))))
        if max_error > 1e-10:
            raise ValueError(f'{side} FK equivalence failed: {max_error}')
        mlo, mhi = (lo-offset)/direction, (hi-offset)/direction
        sdk['T7_170'][side] = dict(alpha=rows[:, 0].tolist(), a=rows[:, 1].tolist(), d=rows[:, 2].tolist(),
            offset=rows[:, 3].tolist(), tool=[0]*6,
            joint_limit_min=np.minimum(mlo, mhi).tolist(), joint_limit_max=np.maximum(mlo, mhi).tolist())
        conventions[side] = dict(joint_names=[j.get('name') for j in joints],
            urdf_from_sdk_direction=direction.tolist(), urdf_from_sdk_offset_rad=offset.tolist(),
            urdf_joint_min_rad=lo.tolist(), urdf_joint_max_rad=hi.tolist(),
            urdf_velocity_rad_s=[float(j.find('limit').get('velocity')) for j in joints],
            urdf_effort_nm=[float(j.find('limit').get('effort')) for j in joints])
        frames[side] = dict(waist_y_from_sdk_base=clean(base), sdk_flange_from_urdf_wrist=clean(bridge),
            sdk_flange_from_urdf_hand=clean(bridge @ origin(nodes[prefix+'_HAND'])))
        ideal_rows = rows.copy()
        ideal_rows[:, 0] = ALPHA
        ideal_rows[:, 1] = 0
        ideal_rows[:, 3] = OFFSET
        ideal_rows[[1, 3, 5, 6], 2] = 0
        ideal_bridge = np.linalg.inv(base @ fk_mdh(ideal_rows, q0)) @ fk_urdf(joints, np.zeros(7))
        # Quantify what rounding the tilted axes/short wrist offset away loses.
        ideal_position_error = ideal_rotation_error = 0.
        for u in samples:
            reference_t = fk_urdf(joints, u)
            ideal_t = base @ fk_mdh(ideal_rows, (u-offset)/direction) @ ideal_bridge
            ideal_position_error = max(ideal_position_error, float(np.linalg.norm(reference_t[:3, 3]-ideal_t[:3, 3])))
            ideal_rotation_error = max(ideal_rotation_error, float(np.linalg.norm(reference_t[:3, :3]-ideal_t[:3, :3])))
        evidence[side] = dict(samples=len(samples), max_transform_element_error=max_error,
            max_error_if_axis_details_rounded_away_m=ideal_position_error,
            max_rotation_matrix_error_if_rounded=ideal_rotation_error,
            ideal_t7_geometry=bool(np.max(np.abs(rows-ideal_rows)) < 1e-10))
    metadata = dict(schema_version=1, model_id=model_id, family='t170c', hardware_version='2.0',
        rated_payload_kg=rated_payload_kg, sdk_model_key='T7_170', coordinates='sdk_mdh_model_rad',
        urdf_file='robot.urdf', urdf_sha256=hashlib.sha256(urdf_bytes).hexdigest(),
        sdk_config_file='Robot_Model.yaml', conventions_file='joint_conventions.yaml', frames_file='frames.yaml',
        tool_policy='per_robot_required', validation='offline_fk_equivalence_only',
        native_sdk_validation='pending', analytic_fixed_j2_validation='pending',
        urdf_meshes='External CAD package; this copy is for geometry, not a standalone visualization package.',
        evidence=evidence)
    return sdk, conventions, frames, metadata


def write_profile(urdf, output, model_id, payload):
    raw = Path(urdf).read_bytes()
    sdk, conventions, frames, metadata = derive(raw, model_id, payload)
    output = Path(output)
    if output.exists():
        raise ValueError('Output exists; generate a new review directory before replacing a model')
    output.mkdir(parents=True)
    (output/'robot.urdf').write_bytes(raw)
    for name, value in [('Robot_Model.yaml', sdk), ('joint_conventions.yaml', conventions), ('frames.yaml', frames)]:
        (output/name).write_text(yaml.safe_dump(value, sort_keys=False, allow_unicode=True))
    metadata['sdk_config_sha256'] = hashlib.sha256((output/'Robot_Model.yaml').read_bytes()).hexdigest()
    (output/'model.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True))
    return metadata


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--model-id', required=True)
    p.add_argument('--payload-kg', type=int, choices=[3, 5], required=True)
    a = p.parse_args()
    print(yaml.safe_dump(write_profile(a.urdf, a.output, a.model_id, a.payload_kg), sort_keys=False))
