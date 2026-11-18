"""Materialize an explicit robot profile without binding it to a machine.
No camera/SDK/motor imports. Legacy application files retain their byte identity.
"""
from pathlib import Path
import hashlib,json,os,shutil,tempfile
import yaml
from .arm_model import prepare_arm_model,legacy_flange_bridge,multiply

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def resolve(root, manifest, output):
    root=Path(root).resolve();manifest=Path(manifest).resolve();output=Path(output).absolute()
    if output.exists():raise ValueError('Output already exists; use a new run directory, never overwrite live calibration/config')
    if output==root or root in output.parents:raise ValueError('Runtime directory must be outside source repository')
    doc=yaml.safe_load(manifest.read_text())
    required={'schema_version','robot_id','robot_model','site_id','task_assignment','layers','files','calibration_directory'}
    if set(doc)!=required or doc['schema_version']!=1:raise ValueError('Unsupported deployment schema/keys')
    if doc['task_assignment']!='dispatch':raise ValueError('Workcells must be assigned by dispatch, not bound to a robot')
    def source(name):
        p=(root/name).resolve()
        if root not in p.parents or not p.exists():raise ValueError('Missing/outside source: '+str(name))
        return p
    merged={};provenance={};inputs={}
    def merge(dst,src,prefix,filename):
        if not isinstance(src,dict):raise ValueError('Layer must contain a mapping')
        for k,v in src.items():
            key=f'{prefix}.{k}' if prefix else k
            if isinstance(v,dict):
                if k in dst and not isinstance(dst[k],dict):raise ValueError('Type conflict: '+key)
                merge(dst.setdefault(k,{}),v,key,filename)
            else:
                if k in dst:raise ValueError('Ambiguous duplicate ownership: '+key)
                dst[k]=v;provenance[key]=filename
    for name in doc['layers']:
        p=source(name);inputs[name]=sha(p);layer=yaml.safe_load(p.read_text())
        if name.startswith('config/robots/'):
            parent='config/robots/'+doc['robot_id']+'/'
            if name==parent+'identity.yaml':
                valid=set(layer)=={'chassis'} and set(layer['chassis'])=={'host','robot_type','sequence_number'}
            elif name==parent+'taught_poses.yaml':
                valid=set(layer)=={'retract_motion'} and set(layer['retract_motion'])=={'ready_right_deg','ready_left_deg','joint_coordinates'} and layer['retract_motion']['joint_coordinates']=='sdk_mdh_model'
            elif name==parent+'reference_pixels.yaml':
                valid=set(layer)<= {'cameras','visual_servo'} and set(layer.get('cameras',{}))<= {
                    'left_gripper_u','left_gripper_v','right_gripper_u','right_gripper_v'}
                for profile,values in layer.get('visual_servo',{}).items():
                    valid=valid and profile in {'left_grasp','right_grasp','left_place','right_place'} and set(values)=={'reference_px'}
            else:valid=False
            if not valid:raise ValueError('Invalid robot identity/calibration ownership; never bind workcells/stations to a robot')
        merge(merged,layer,'',name)
    if merged.get('chassis',{}).get('sequence_number')!=doc['robot_id']:raise ValueError('Robot identity mismatch')
    arm_resources, arm_inputs, selected_arm_model = prepare_arm_model(root,doc['robot_model'],doc['robot_id'],merged.get('retract_motion'))
    inputs.update(arm_inputs)
    files=doc['files']
    if set(files) & set(arm_resources):raise ValueError('Selected arm model owns its runtime files; remove conflicting manual mappings')
    for target,name in files.items():
        if Path(target).is_absolute() or '..' in Path(target).parts or target in ['config/move_box_params.yaml','config/calibration']:raise ValueError('Invalid runtime target '+target)
        p=source(name)
        if not p.is_file():raise ValueError('Mapped resource must be a file: '+name)
        inputs[name]=sha(p)
    calibration=source(doc['calibration_directory'])
    missing_task_calibrations=[]
    for task in ['grasp','placement']:
        for hand in ['left','right']:
            p=calibration/task/(hand+'.yaml')
            if not p.exists():
                missing_task_calibrations.append(task+'/'+hand+'.yaml')
                continue # New robot maintenance/bootstrap: never copy another robot's results.
            text=p.read_text()
            data=json.loads(text) if text.lstrip().startswith('{') else yaml.safe_load(text)
            if data.get('robot_id')!=doc['robot_id']:raise ValueError('Calibration belongs to another robot')
            inputs[str(p.relative_to(root))]=sha(p)
    output.parent.mkdir(parents=True,exist_ok=True)
    temp=Path(tempfile.mkdtemp(prefix='.t170-profile-',dir=output.parent))
    try:
        # Only directories with substituted files need an overlay. Other assets
        # stay referenced from the source tree until installation is implemented.
        def overlay(src,dst,prefix=''):
            dst.mkdir(exist_ok=True)
            for child in src.iterdir():
                rel=f'{prefix}/{child.name}'.lstrip('/');target=dst/child.name
                if rel in ['.git','config','logs','debug_picture','reports']:continue
                if rel in files:target.write_bytes(source(files[rel]).read_bytes())
                elif child.is_dir() and any(k.startswith(rel+'/') for k in files):overlay(child,target,rel)
                else:target.symlink_to(child.resolve(),target_is_directory=child.is_dir())
        overlay(root,temp)
        for target,name in files.items():
            p=temp/target
            if not p.exists():p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(source(name).read_bytes())
        for target,data in arm_resources.items():
            p=temp/target
            p.parent.mkdir(parents=True,exist_ok=True)
            p.write_bytes(data)
        # Re-express existing eye-in-hand transforms in the exact selected flange
        # coordinates. Original robot calibration files remain immutable inputs.
        binding=yaml.safe_load(arm_resources['config/arm_binding.yaml'])
        if binding.get('hand_eye_coordinates')=='legacy_t7_flange':
            frame_data=yaml.safe_load(arm_resources['config/arm_frames.yaml'])
            for side in ['left','right']:
                p=temp/'config'/(side+'_hand_to_base_result.yaml')
                eye=yaml.safe_load(p.read_text());key='a. matrix' if 'a. matrix' in eye else 'matrix'
                bridge=legacy_flange_bridge(frame_data,side)
                for i in range(3):bridge[i][3]*=1000 # hand-eye source translations are mm
                eye[key]=multiply(bridge,eye[key])
                for k in ['b. translation_xyz_mm','c. euler_angles_xyz_deg']:eye.pop(k,None)
                eye['parent_frame']='selected_sdk_flange';eye['source_parent_frame']='legacy_t7_flange'
                p.write_text(yaml.safe_dump(eye,sort_keys=False,allow_unicode=True))
        (temp/'config').mkdir(exist_ok=True)
        (temp/'config/move_box_params.yaml').write_text(yaml.safe_dump(merged,allow_unicode=True,sort_keys=False))
        # Directory link preserves atomic calibration saves and *_before_ history
        # inside the selected robot's writable calibration package.
        (temp/'config/calibration').symlink_to(calibration,target_is_directory=True)
        for folder in ['logs','debug_picture','reports']:(temp/folder).mkdir()
        # Temporary path aliases for the unchanged Web service only. Canonical
        # source code and manifest use config/vision, config/head and assets.
        web_aliases = {
            "board_yf100_aruco/config.yaml": "config/vision/tray.yaml",
            "board_belt_aruco/config.yaml": "config/vision/belt.yaml",
            "online_pose/config.yaml": "config/vision/marker.yaml",
            "feeding_cylindrical_parts_alg/config/pose_params.yaml": "config/vision/pose_params.yaml",
            "feeding_cylindrical_parts_alg/models": "assets/models",
            "free_calib/config/head_params.yaml": "config/head/geometry.yaml",
            "free_calib/config/camera_to_head_connector_result.yaml": "config/head/extrinsics.yaml",
            "free_calib/config/extrinsics.yaml": "config/head/extrinsics.yaml",
        }
        for old, canonical in web_aliases.items():
            target = temp / canonical
            if not target.exists(): continue
            alias = temp / old
            alias.parent.mkdir(parents=True, exist_ok=True)
            alias.symlink_to(os.path.relpath(target, alias.parent), target_is_directory=target.is_dir())
        (temp/"web_compat_paths.json").write_text(json.dumps(web_aliases,indent=2)+"\n")
        record={'schema_version':1,'manifest':str(manifest),'robot_id':doc['robot_id'],'robot_model':doc['robot_model'],'site_id':doc['site_id'],'task_assignment':'dispatch','parameter_sources':provenance,'input_sha256':inputs,'resource_root':str(root),'runtime_root':str(output),'mode':'source-tree-overlay','missing_task_calibrations':missing_task_calibrations}
        if selected_arm_model is not None:record['selected_arm_model']=selected_arm_model
        (temp/'deployment_resolved.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
        temp.rename(output)
    except BaseException:
        shutil.rmtree(temp);raise
    return record
