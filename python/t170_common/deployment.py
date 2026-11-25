"""Materialize an explicit robot profile without binding it to a machine.
No camera/SDK/motor imports. Legacy application files retain their byte identity.
"""
from pathlib import Path
import hashlib,json,os,shutil,tempfile
import yaml
from .arm_model import prepare_arm_model
from .camera_nominal import hand_eye_for_servo, image_rotations
from .parameter_catalog import QUICK_FIELDS, quick_key_allowed

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
                if k in dst:raise ValueError('Ambiguous duplicate ownership: '+key+' in '+provenance.get(key,'another layer')+' and '+filename)
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
            elif name==parent+'quick_params.yaml':
                def keys(value,prefix=''):
                    if isinstance(value,dict):
                        return {k for key,v in value.items() for k in keys(v,(prefix+'.'+str(key)).strip('.'))}
                    if isinstance(value,list):
                        return {k for i,v in enumerate(value) for k in keys(v,prefix+'.'+str(i))}
                    return {prefix}
                present=keys(layer)
                if not QUICK_FIELDS<=present:raise ValueError('quick_params.yaml missing required fields: '+', '.join(sorted(QUICK_FIELDS-present)))
                valid=isinstance(layer,dict) and all(quick_key_allowed(key) for key in present)
            else:valid=False
            if not valid:raise ValueError('Invalid robot identity/calibration ownership; never bind workcells/stations to a robot')
        merge(merged,layer,'',name)
    # Manual task station defaults come from one site-owned group table.
    # Dispatch still supplies and scopes the actual production task points.
    from .maintenance_workcells import RUNTIME_FILE, STATION_FIELDS, load_catalog, selected_stations
    maintenance_name=doc['files'].get(RUNTIME_FILE)
    if not maintenance_name:raise ValueError('档案缺少 maintenance_workcells.yaml；请先运行升级迁移')
    maintenance=load_catalog(source(maintenance_name))
    inputs[maintenance_name]=sha(source(maintenance_name))
    stations=selected_stations(maintenance)
    for key in STATION_FIELDS:
        if key in merged.get('chassis',{}):
            raise ValueError('维护站点已归机床组配置管理，请从旧参数层移除 chassis.'+key)
        merged.setdefault('chassis',{})[key]=stations[key]
        provenance['chassis.'+key]=maintenance_name
    if merged.get('chassis',{}).get('sequence_number')!=doc['robot_id']:raise ValueError('Robot identity mismatch')
    arm_resources, arm_inputs, selected_arm_model = prepare_arm_model(root,doc['robot_model'],doc['robot_id'],merged.get('retract_motion'))
    inputs.update(arm_inputs)
    files=doc['files']
    if set(files) & set(arm_resources):raise ValueError('Selected arm model owns its runtime files; remove conflicting manual mappings')
    for target,name in files.items():
        if Path(target).is_absolute() or '..' in Path(target).parts or target in ['config/effective_params.yaml','config/calibration']:raise ValueError('Invalid runtime target '+target)
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
                if rel in ['.git','.t170-config.lock','config','logs','field_logs','debug_picture','reports']:continue
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
        frame_data=yaml.safe_load(arm_resources['config/arm_frames.yaml'])
        rotations=image_rotations(yaml.safe_load((temp/'config/realsense_cameras.yaml').read_text()))
        for side in ['left','right']:
            p=temp/'config'/(side+'_hand_to_base_result.yaml')
            eye=hand_eye_for_servo(yaml.safe_load(p.read_text()),binding,frame_data,side,rotations[side])
            p.write_text(yaml.safe_dump(eye,sort_keys=False,allow_unicode=True))
        (temp/'config').mkdir(exist_ok=True)
        (temp/'config/effective_params.yaml').write_text(yaml.safe_dump(merged,allow_unicode=True,sort_keys=False))
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
            "free_calib/config/camera_to_head_connector_result.yaml": "config/head/"+yaml.safe_load((temp/'config/head/geometry.yaml').read_text()).get('camera_to_head_yaml','extrinsics.yaml'),
            "free_calib/config/extrinsics.yaml": "config/head/"+yaml.safe_load((temp/'config/head/geometry.yaml').read_text()).get('camera_to_head_yaml','extrinsics.yaml'),
        }
        for old, canonical in web_aliases.items():
            target = temp / canonical
            if not target.exists(): continue
            alias = temp / old
            alias.parent.mkdir(parents=True, exist_ok=True)
            alias.symlink_to(os.path.relpath(target, alias.parent), target_is_directory=target.is_dir())
        (temp/"web_compat_paths.json").write_text(json.dumps(web_aliases,indent=2)+"\n")
        record={'schema_version':1,'manifest':str(manifest),'robot_id':doc['robot_id'],'robot_model':doc['robot_model'],'site_id':doc['site_id'],'task_assignment':'dispatch','parameter_sources':provenance,'input_sha256':inputs,'resource_root':str(root),'runtime_root':str(output),'mode':'source-tree-overlay','missing_task_calibrations':missing_task_calibrations}
        record['maintenance_workcells']=maintenance
        if selected_arm_model is not None:record['selected_arm_model']=selected_arm_model
        (temp/'deployment_resolved.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
        temp.rename(output)
    except BaseException:
        shutil.rmtree(temp);raise
    return record
