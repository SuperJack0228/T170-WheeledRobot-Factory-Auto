"""Real loader equivalence and robot/workcell ownership, without any robot IO."""
from pathlib import Path
import unittest,tempfile,sys,subprocess,copy,json,shutil
import yaml,os
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
  shutil.copyfile(ROOT/'tests/fixtures/no124_factory_baseline.yaml',out/'config/baseline.yaml')
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
  self.doc['files']['../escaped']='config/robot_models/t170/Robot_Arm_Model.yaml'
  with self.assertRaises(ValueError):resolve(ROOT,self.manifest(),self.base/'run')
 def test_launcher_passes_resolved_paths_without_real_factory(self):
  fake=self.base/'source';fake.mkdir()
  shutil.copytree(ROOT/'config',fake/'config')
  (fake/'python').symlink_to(ROOT/'python',target_is_directory=True)
  for folder in ['deploy','tools','build_robot']:(fake/folder).mkdir()
  shutil.copyfile(ROOT/'deploy/run_deployment.sh',fake/'deploy/run_deployment.sh')
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
