"""Real loader equivalence and robot/workcell ownership, without any robot IO."""
from pathlib import Path
import unittest,tempfile,sys,subprocess,copy,json,shutil
import yaml,os
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from t170_common.deployment import resolve
DUMP=sys.argv.pop(1) if len(sys.argv)>1 else None
class DeploymentTest(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.base=Path(self.tmp.name)
  self.doc=yaml.safe_load((ROOT/'config/deployments/TH-T170C-0124.yaml').read_text())
 def tearDown(self):self.tmp.cleanup()
 def manifest(self):
  p=self.base/'deployment.yaml';p.write_text(yaml.safe_dump(self.doc));return p
 def test_real_loader_equivalence(self):
  out=self.base/'run';record=resolve(ROOT,self.manifest(),out)
  self.assertEqual(record['task_assignment'],'dispatch')
  # Compare effective controls after explicitly normalizing the old device-angle
  # fixture to the new coordinate contract. Physical ready targets are checked
  # independently below; a string comparison cannot compare unlike coordinates.
  baseline=yaml.safe_load((ROOT/'tests/fixtures/no124_factory_baseline.yaml').read_text())
  current=yaml.safe_load((out/'config/move_box_params.yaml').read_text())
  for key in ['ready_right_deg','ready_left_deg','joint_coordinates']:
   baseline['retract_motion'][key]=current['retract_motion'][key]
  (out/'config/baseline.yaml').write_text(yaml.safe_dump(baseline))
  for alias in json.loads((out/'web_compat_paths.json').read_text()):
   self.assertTrue((out/alias).exists(),alias)
  subprocess.run([DUMP,'--compare',str(out/'config/baseline.yaml'),str(out/'config/move_box_params.yaml')],check=True,capture_output=True)
  # Detect an effective, safety-relevant change rather than just file hashes.
  p=out/'config/move_box_params.yaml';data=yaml.safe_load(p.read_text());data['head_grasp']['grasp_z_offset_m']+=.01;p.write_text(yaml.safe_dump(data))
  self.assertEqual(subprocess.run([DUMP,'--compare',str(out/'config/baseline.yaml'),str(p)],capture_output=True).returncode,1)
  self.assertTrue((out/'config/calibration').is_symlink())
  self.assertEqual((out/'config/calibration').resolve(),ROOT/self.doc['calibration_directory'])
  self.assertIn('conveyor.grasp_rpy_deg.right',record['parameter_sources'])
  with self.assertRaises(ValueError):resolve(ROOT,self.manifest(),out)
 def test_source_launcher_materializes_selected_profile(self):
  fake=self.base/'source';fake.mkdir()
  shutil.copytree(ROOT/'config',fake/'config')
  (fake/'python').symlink_to(ROOT/'python',target_is_directory=True)
  for folder in ['deploy','tools','build_robot']:(fake/folder).mkdir()
  for name in ['run_factory.sh','runtime_context.sh']:
   shutil.copyfile(ROOT/'deploy'/name,fake/'deploy'/name)
  shutil.copyfile(ROOT/'tools/resolve_deployment.py',fake/'tools/resolve_deployment.py')
  (fake/'build_robot/t170_config_dump').symlink_to(Path(DUMP).resolve())
  app=fake/'build_robot/t170_factory'
  app.write_text('#!/bin/bash\ntest -f "$T170_CONFIG_DIR/move_box_params.yaml" || exit 9\nprintf "%s\\n" "$T170_ROOT" "$T170_CONFIG_DIR" "$@" > "$T170_ROOT/launch_record.txt"\n')
  app.chmod(0o755)
  env=dict(os.environ,T170_ROOT=str(fake),T170_PYTHON=sys.executable,T170_STATE_DIR=str(self.base/'state'))
  for key in ['T170_RUNTIME_DIR','T170_CONFIG_DIR','T170_DEPLOYMENT']:env.pop(key,None)
  subprocess.run(['bash',str(fake/'deploy/run_factory.sh'),'--maintenance'],env=env,cwd='/',check=True,capture_output=True)
  runtime=next((self.base/'state').glob('session.*/runtime'))
  self.assertTrue((runtime/'effective_config.yaml').is_file())
  self.assertEqual((runtime/'launch_record.txt').read_text().splitlines(),[str(runtime),str(runtime/'config'),'--maintenance'])
  self.assertEqual(json.loads((runtime/'deployment_resolved.json').read_text())['robot_id'],'TH-T170C-0124')
 def test_wrong_robot_rejected(self):
  self.doc['robot_id']='another-robot'
  with self.assertRaisesRegex(ValueError,'identity'):resolve(ROOT,self.manifest(),self.base/'run')
 def test_arm_variants_and_per_robot_tcp_are_materialized(self):
  for model_id,shoulder,forearm in [('t170c_v2_3kg',.192,.252),('t170c_v2_5kg',.2,.271)]:
   with self.subTest(model=model_id):
    self.doc['robot_model']=model_id
    out=self.base/model_id;record=resolve(ROOT,self.manifest(),out)
    sdk=yaml.safe_load((out/'config/Robot_Model.yaml').read_text())['T7_170']
    self.assertAlmostEqual(sdk['right']['d'][0],shoulder)
    self.assertAlmostEqual(sdk['left']['d'][0],-shoulder)
    self.assertAlmostEqual(sdk['right']['d'][4],forearm)
    self.assertAlmostEqual(sdk['left']['tool'][0],.26)
    if model_id.endswith('3kg'):self.assertGreater(abs(sdk['left']['tool'][3]),.002)
    self.assertEqual(record['selected_arm_model']['model_id'],model_id)
    self.assertIn('config/robots/TH-T170C-0124/arm_tools.yaml',record['input_sha256'])
    self.assertFalse((out/'config/Robot_Arm_Model.yaml').exists())
    self.assertEqual(yaml.safe_load((out/'config/arm_binding.yaml').read_text())['model_id'],model_id)
    binding=yaml.safe_load((out/'config/arm_binding.yaml').read_text())
    policy=yaml.safe_load((ROOT/'config/robot_models'/model_id/'motion_policy.yaml').read_text())
    self.assertEqual(binding['motion_policy'],policy)
    self.assertIn('config/robot_models/'+model_id+'/motion_policy.yaml',record['input_sha256'])
    self.assertEqual((out/'config/arm_model.urdf').read_bytes(),
      (ROOT/'config/robot_models'/model_id/'robot.urdf').read_bytes())
 def test_arm_model_identity_hash_and_ownership_rejected(self):
  self.doc['robot_model']='t170c_v2_8kg'
  with self.assertRaisesRegex(ValueError,'Unknown robot_model'):resolve(ROOT,self.manifest(),self.base/'unknown')
  self.doc['robot_model']='t170c_v2_3kg'
  self.doc['files']['config/Robot_Model.yaml']='config/robot_models/t170c_v2_5kg/Robot_Model.yaml'
  with self.assertRaisesRegex(ValueError,'conflicting'):resolve(ROOT,self.manifest(),self.base/'mixed')
  del self.doc['files']['config/Robot_Model.yaml']
  fake=self.base/'source';shutil.copytree(ROOT/'config',fake/'config')
  urdf=fake/'config/robot_models/t170c_v2_3kg/robot.urdf'
  urdf.write_bytes(urdf.read_bytes()+b'\n<!-- changed geometry source -->\n')
  with self.assertRaisesRegex(ValueError,'urdf hash mismatch'):resolve(fake,self.manifest(),self.base/'stale')
  self.assertFalse((self.base/'stale').exists())
 def test_external_sdk_yaml_edit_is_loaded_without_reusing_validation(self):
  fake=self.base/'source';shutil.copytree(ROOT/'config',fake/'config')
  model=fake/'config/robot_models/t170c_v2_3kg/Robot_Model.yaml'
  data=yaml.safe_load(model.read_text());data['T7_170']['right']['d'][4]+=.0005
  model.write_text(yaml.safe_dump(data))
  out=self.base/'modified';record=resolve(fake,self.manifest(),out)
  loaded=yaml.safe_load((out/'config/Robot_Model.yaml').read_text())
  self.assertAlmostEqual(loaded['T7_170']['right']['d'][4],.2525)
  selected=record['selected_arm_model']
  self.assertFalse(selected['sdk_config_matches_derived_profile'])
  self.assertEqual(selected['validation'],'sdk_yaml_modified_not_equivalence_checked')
  self.assertNotIn('evidence',selected)
 def test_ready_targets_and_tcp_camera_relation_preserve_physical_frames(self):
  from t170_common.arm_model import pose_matrix
  out=self.base/'frame-test';resolve(ROOT,self.manifest(),out)
  config=yaml.safe_load((out/'config/move_box_params.yaml').read_text())
  binding=yaml.safe_load((out/'config/arm_binding.yaml').read_text())
  sdk=yaml.safe_load((out/'config/Robot_Model.yaml').read_text())['T7_170']
  before=yaml.safe_load((ROOT/'tests/fixtures/no124_factory_baseline.yaml').read_text())
  for side in ['left','right']:
   degrees=np.array(config['retract_motion']['ready_'+side+'_deg'])
   actual=degrees*binding[side]['model_to_device_direction']+np.rad2deg(binding[side]['model_to_device_offset_rad'])
   np.testing.assert_allclose(actual,before['retract_motion']['ready_'+side+'_deg'],rtol=0,atol=1e-10)
   old=yaml.safe_load((ROOT/f'config/calibration/TH-T170C-0124/hand_eye/{side}.yaml').read_text())
   new=yaml.safe_load((out/f'config/{side}_hand_to_base_result.yaml').read_text())
   eye0=np.array(old['a. matrix']);eye1=np.array(new['a. matrix']);eye0[:3,3]*=.001;eye1[:3,3]*=.001
   tcp0=np.array(pose_matrix([.26,0,0,0,0,0]));tcp1=np.array(pose_matrix(sdk[side]['tool']))
   np.testing.assert_allclose(np.linalg.inv(tcp0)@eye0,np.linalg.inv(tcp1)@eye1,rtol=0,atol=1e-12)
 def test_missing_task_calibrations_allow_maintenance_bootstrap(self):
  fake=self.base/'source';shutil.copytree(ROOT/'config',fake/'config')
  directory=fake/self.doc['calibration_directory']
  for file in directory.glob('*/*.yaml'):file.unlink()
  out=self.base/'bootstrap';record=resolve(fake,self.manifest(),out)
  self.assertEqual(len(record['missing_task_calibrations']),4)
  self.assertEqual(list((out/'config/calibration').glob('*/*.yaml')),[])
  subprocess.run([DUMP,str(out/'config/move_box_params.yaml'),str(out/'effective.yaml')],check=True,capture_output=True)
 def test_parameter_index_locates_canonical_owner(self):
  result=subprocess.run([sys.executable,str(ROOT/'tools/parameter_index.py'),'visual_servo.kp'],check=True,capture_output=True,text=True)
  self.assertIn('visual_servo.kp = ',result.stdout)
  self.assertIn(str(ROOT/'config/defaults/factory_baseline.yaml'),result.stdout)
 def test_no_fixed_robot_workcell_binding(self):
  robot=yaml.safe_load((ROOT/'config/robots/TH-T170C-0124/identity.yaml').read_text())
  self.assertEqual(set(robot['chassis']),{'host','robot_type','sequence_number'})
  self.doc['task_assignment']='machine_1'
  with self.assertRaisesRegex(ValueError,'dispatch'):resolve(ROOT,self.manifest(),self.base/'run')
  self.doc['task_assignment']='dispatch';self.doc['machine_id']='machine_1'
  with self.assertRaisesRegex(ValueError,'schema'):resolve(ROOT,self.manifest(),self.base/'run')
 def test_missing_input_rejected_without_partial_output(self):
  self.doc['layers'].append('config/missing.yaml')
  with self.assertRaises(ValueError):resolve(ROOT,self.manifest(),self.base/'run')
  self.assertFalse((self.base/'run').exists())
 def test_runtime_path_escape_rejected(self):
  self.doc['files']['../escaped']='config/robot_models/t170/Robot_Model.yaml'
  with self.assertRaises(ValueError):resolve(ROOT,self.manifest(),self.base/'run')
 def test_launcher_passes_resolved_paths_without_real_factory(self):
  fake=self.base/'source';fake.mkdir()
  shutil.copytree(ROOT/'config',fake/'config')
  (fake/'python').symlink_to(ROOT/'python',target_is_directory=True)
  for folder in ['deploy','tools','build_robot']:(fake/folder).mkdir()
  shutil.copyfile(ROOT/'deploy/run_deployment.sh',fake/'deploy/run_deployment.sh')
  shutil.copyfile(ROOT/'deploy/runtime_context.sh',fake/'deploy/runtime_context.sh')
  shutil.copyfile(ROOT/'tools/resolve_deployment.py',fake/'tools/resolve_deployment.py')
  (fake/'build_robot/t170_config_dump').symlink_to(Path(DUMP).resolve())
  factory=fake/'build_robot/t170_factory'
  factory.write_text('#!/bin/bash\nprintf "%s\\n" "$T170_ROOT" "$T170_CONFIG_DIR" "$T170_DEPLOYMENT" "$PWD" "$@" > "$T170_ROOT/launch_record.txt"\n')
  factory.chmod(0o755)
  out=self.base/'launched'
  env=dict(os.environ,T170_PYTHON=sys.executable)
  subprocess.run(['bash',str(fake/'deploy/run_deployment.sh'),str(fake/'config/deployments/TH-T170C-0124.yaml'),str(out),'--dispatch'],env=env,check=True,capture_output=True)
  lines=(out/'launch_record.txt').read_text().splitlines()
  self.assertEqual(lines[0],str(out));self.assertEqual(lines[1],str(out/'config'))
  self.assertEqual(lines[3],str(out));self.assertEqual(lines[4],'--dispatch')
  self.assertTrue((out/'effective_config.yaml').is_file())
if __name__=='__main__':unittest.main()
