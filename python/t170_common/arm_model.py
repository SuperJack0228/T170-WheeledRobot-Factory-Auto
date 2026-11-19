"""Prepare a selected vendor-SDK model. Does not load SDKs or enable a backend."""
from pathlib import Path
import copy
import hashlib
import math
import re

import yaml

def multiply(a,b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def pose_matrix(p):
    x,y,z,rx,ry,rz=p;cx,sx,cy,sy,cz,sz=math.cos(rx),math.sin(rx),math.cos(ry),math.sin(ry),math.cos(rz),math.sin(rz)
    return [[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,x],
            [sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,y],[-sy,cy*sx,cy*cx,z],[0,0,0,1]]

def matrix_pose(t):
    ry=math.atan2(-t[2][0],math.hypot(t[0][0],t[1][0]))
    if abs(math.cos(ry))<1e-8:raise ValueError('Tool rotation at Euler singularity; use an explicit SDK-frame pose')
    return [t[0][3],t[1][3],t[2][3],math.atan2(t[2][1],t[2][2]),ry,math.atan2(t[1][0],t[0][0])]

def legacy_flange_bridge(frames,side):
    # Fixed frame rotation only. Arm-length errors are not folded into hand-eye.
    old=[[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]] if side=='left' else [[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]]
    inverse=[[old[j][i] if i<3 and j<3 else float(i==j) for j in range(4)] for i in range(4)]
    return multiply(frames[side]['sdk_flange_from_urdf_wrist'],inverse)


def prepare_arm_model(root, model_id, robot_id, motion_limits=None):
    root = Path(root).resolve()
    if model_id == 't170':
        raise ValueError('Select t170c_v2_3kg or t170c_v2_5kg for the new arm SDK')
    match = re.fullmatch(r't170c_v2_(3|5)kg', model_id) if isinstance(model_id, str) else None
    if match is None:
        raise ValueError('Unknown robot_model: '+str(model_id))
    inputs = {}

    def read(name):
        p = (root/name).resolve()
        if root not in p.parents or not p.is_file():
            raise ValueError('Missing/outside arm model input: '+str(name))
        data = p.read_bytes()
        inputs[str(p.relative_to(root))] = hashlib.sha256(data).hexdigest()
        return data

    def vector(node, size, label):
        if (not isinstance(node, list) or len(node) != size or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in node)):
            raise ValueError('Invalid arm model vector: '+label)

    directory = Path('config/robot_models')/model_id
    meta = yaml.safe_load(read(directory/'model.yaml'))
    if (meta.get('schema_version') != 1 or meta.get('model_id') != model_id or
            meta.get('coordinates') != 'sdk_mdh_model_rad' or meta.get('sdk_model_key') != 'T7_170' or
            meta.get('rated_payload_kg') != int(match.group(1))):
        raise ValueError('Arm model identity/coordinate mismatch')
    raw_inputs = {}
    for key in ['urdf_file', 'sdk_config_file', 'conventions_file', 'frames_file']:
        filename = meta.get(key)
        if not isinstance(filename, str) or Path(filename).name != filename or filename in ('.', '..'):
            raise ValueError('Invalid arm model resource path')
        raw_inputs[key] = read(directory/filename)
    if hashlib.sha256(raw_inputs['urdf_file']).hexdigest() != meta.get('urdf_sha256'):
        raise ValueError('Arm model urdf hash mismatch; regenerate/review the profile')
    sdk_source_hash = hashlib.sha256(raw_inputs['sdk_config_file']).hexdigest()
    matches_derived_profile = sdk_source_hash == meta.get('sdk_config_sha256')
    sdk = yaml.safe_load(raw_inputs['sdk_config_file'])
    model = sdk[meta['sdk_model_key']]
    if model.get('robot_dof') != 7:
        raise ValueError('Expected dual seven-axis arm model')
    for side in ['left', 'right']:
        for key in ['alpha', 'a', 'd', 'offset', 'joint_limit_min', 'joint_limit_max']:
            vector(model[side][key], 7, side+'.'+key)
        if any(a >= b for a, b in zip(model[side]['joint_limit_min'], model[side]['joint_limit_max'])):
            raise ValueError('Invalid joint limit ordering')
    period = sdk['_plan']['sample_rate']
    if isinstance(period, bool) or not isinstance(period, (int, float)) or not math.isfinite(period) or period <= 0:
        raise ValueError('Invalid arm planning period')

    # TCP belongs to a physical robot/tool installation, not the arm payload
    # rating. Never take the URDF HAND mount origin as a grasp TCP implicitly.
    tool_path = Path('config/robots')/robot_id/'arm_tools.yaml'
    tools = yaml.safe_load(read(tool_path))
    if tools.get('schema_version') != 1 or tools.get('coordinates') not in ['sdk_flange_xyz_rpy_m_rad','legacy_t7_flange_xyz_rpy_m_rad']:
        raise ValueError('Invalid per-robot arm tool coordinates')
    frame_data=yaml.safe_load(raw_inputs['frames_file'])
    for side in ['left', 'right']:
        vector(tools.get(side), 6, 'tools.'+side)
        model[side]['tool'] = matrix_pose(multiply(legacy_flange_bridge(frame_data,side),pose_matrix(tools[side]))) if tools['coordinates'].startswith('legacy_') else tools[side]
    binding_path=Path('config/robots')/robot_id/'arm_binding.yaml'
    binding=yaml.safe_load(read(binding_path))
    if binding.get('schema_version')!=1 or binding.get('joint_coordinates')!='sdk_mdh_model_rad':raise ValueError('Invalid arm binding coordinates')
    if binding.get('hand_eye_coordinates') not in ['legacy_t7_flange','sdk_flange']:raise ValueError('Unknown hand-eye parent frame')
    all_ids=[]
    for side in ['left','right']:
        b=binding[side]
        vector(b.get('model_to_device_direction'),7,'binding direction');vector(b.get('model_to_device_offset_rad'),7,'binding offset')
        if any(v not in [-1,1] for v in b['model_to_device_direction']):raise ValueError('Invalid direction')
        ids=b.get('can_ids')
        if not isinstance(ids,list) or len(ids)!=7 or any(type(v)!=int or not 1<=v<=127 for v in ids):raise ValueError('Invalid motor IDs')
        all_ids+=ids
    if len(set(all_ids))!=14:raise ValueError('Duplicate arm motor IDs')
    binding['model_id']=model_id
    policy=yaml.safe_load(read(directory/'motion_policy.yaml'))
    if policy.get('schema_version')!=1 or policy.get('joint_coordinates')!='sdk_mdh_model_deg':
        raise ValueError('Invalid arm motion policy')
    for side in ['left','right']:
        for key in ['home_deg','preferred_deg']:
            vector(policy[side].get(key),7,'motion_policy.'+side+'.'+key)
            if any(not low<=v*math.pi/180<=high for low,v,high in zip(
                    model[side]['joint_limit_min'],policy[side][key],model[side]['joint_limit_max'])):
                raise ValueError('Motion policy outside selected arm model limits')
    for key in ['posture_gain','elbow_gain','limit_gain','elbow_outward_ratio']:
        v=policy.get(key)
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<0:
            raise ValueError('Invalid arm motion policy gain: '+key)
    binding['motion_policy']=policy
    limits=motion_limits or {}
    binding['trajectory_limits']={name:limits[key] for name,key in [('velocity_rad_s','max_joint_velocity_rad_s'),('acceleration_rad_s2','max_joint_acceleration_rad_s2'),('jerk_rad_s3','max_joint_jerk_rad_s3')]}
    rendered = yaml.safe_dump(sdk, sort_keys=False).encode()
    selected = copy.deepcopy(meta)
    selected['sdk_config_loaded_sha256'] = sdk_source_hash
    selected['sdk_config_matches_derived_profile'] = matches_derived_profile
    if not matches_derived_profile:
        # YAML is an external tuning input. Accept an intentional edit, but do
        # not relabel it as the FK-equivalent profile previously tested.
        selected['original_derived_profile_evidence'] = selected.pop('evidence', {})
        selected['validation'] = 'sdk_yaml_modified_not_equivalence_checked'
    selected['robot_id'] = robot_id
    selected['tool_source'] = str(tool_path)
    selected['tool_validation'] = tools.get('validation', 'unverified')
    selected['prepared_config_sha256'] = hashlib.sha256(rendered).hexdigest()
    selected['scope'] = 'Active Ti5_Robotics deployment; joints are SDK model radians.'
    selected['binding_source']=str(binding_path)
    selected['tool_source_coordinates']=tools['coordinates']
    resources = {
        'config/Robot_Model.yaml': rendered,
        'config/arm_model_metadata.yaml': yaml.safe_dump(selected, sort_keys=False).encode(),
        'config/arm_model.urdf': raw_inputs['urdf_file'],
        'config/arm_joint_conventions.yaml': raw_inputs['conventions_file'],
        'config/arm_frames.yaml': raw_inputs['frames_file'],
        'config/arm_binding.yaml': yaml.safe_dump(binding,sort_keys=False).encode(),
    }
    return resources, inputs, selected
