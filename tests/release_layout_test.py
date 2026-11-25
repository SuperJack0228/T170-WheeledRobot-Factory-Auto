"""Install shared CMake resources, relocate them and boot a fake application."""
from pathlib import Path
import json
import os
import shutil
import subprocess
import socket
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import importlib.util
import yaml

ROOT = Path(__file__).resolve().parents[1]
CHECKER = Path(sys.argv.pop(1)).resolve()
spec = importlib.util.spec_from_file_location('release_dependency_check', ROOT / 'packaging/check_runtime.py')
dependency_check = importlib.util.module_from_spec(spec); spec.loader.exec_module(dependency_check)


class NativeDependencyPolicyTest(unittest.TestCase):
    def test_installed_dependency_policy(self):
        stage = Path('/tmp/installed package'); source = Path('/tmp/source')
        for output, accepted in [
            ('libarm.so => /tmp/installed package/lib/libarm.so (0x1000)\n', True),
            ('libarm.so => /tmp/source/dependencies/libarm.so (0x1000)\n', False),
            ('libarm.so => not found\n', False),
        ]:
            with self.subTest(output=output), patch.object(dependency_check.subprocess, 'run',
                    return_value=SimpleNamespace(returncode=0, stdout=output, stderr='')):
                if accepted: self.assertTrue(dependency_check.check(stage, source))
                else:
                    with self.assertRaises(ValueError): dependency_check.check(stage, source)


class InstalledLayoutTest(unittest.TestCase):
    def test_relocated_package_has_persistent_robot_configuration(self):
        self.check_relocated_package(external=False)

    def test_explicit_external_workspace_remains_supported(self):
        self.check_relocated_package(external=True)

    def check_relocated_package(self, external):
        with tempfile.TemporaryDirectory(prefix='t170-installed-') as temporary:
            base = Path(temporary); fixture = base / 'fixture'; fixture.mkdir()
            fake = fixture / 't170_factory'
            fake.write_text('#!' + sys.executable + '\nimport os,json\nfrom pathlib import Path\np=Path(os.environ["T170_ROOT"]);(p/"launch_record.json").write_text(json.dumps(dict(os.environ)))\n')
            fake.chmod(0o755)
            (fixture / 'release.json').write_text('{"version":"layout-test","test_fixture":true}\n')
            cmake = f'''cmake_minimum_required(VERSION 3.16)
project(layout_test NONE)
set(T170_PACKAGING_RESOURCES_ONLY ON)
include("{ROOT}/cmake/T170Packaging.cmake")
t170_install_resources("{ROOT}")
install(PROGRAMS "{fake}" "{CHECKER}" DESTINATION bin)
install(FILES "{fixture}/release.json" DESTINATION .)
set(CPACK_PACKAGE_NAME layout-test)
set(CPACK_PACKAGE_VERSION 0.0.0)
set(CPACK_GENERATOR TGZ)
include(CPack)
'''
            (fixture / 'CMakeLists.txt').write_text(cmake)
            build = base / 'build'; install = base / 'install'
            subprocess.run(['cmake','-S',str(fixture),'-B',str(build)],check=True,capture_output=True)
            subprocess.run(['cmake','--install',str(build),'--prefix',str(install)],check=True,capture_output=True)
            subprocess.run(['cpack','--config',str(build/'CPackConfig.cmake'),'-B',str(base/'archive')],check=True,capture_output=True)
            package = base / 'relocated package'; install.rename(package); shutil.rmtree(fixture); shutil.rmtree(build)
            self.assertFalse((package / 'src').exists())
            self.assertFalse((package / 'assets/models/archive').exists())
            self.assertTrue((package / 'assets/models/best1103.pt').is_file())
            self.assertTrue((package / 'tools/upgrade_robot.py').is_file())
            self.assertTrue((package / 'deploy/upgrade_robot.sh').is_file())
            self.assertTrue((package / 'docs/V1_RELEASE_AND_FIRST_DEPLOYMENT.md').is_file())
            self.assertTrue((package / 'docs/RELEASE_NOTES_1.0.0.md').is_file())
            self.assertTrue((package / 'config').is_dir())
            self.assertFalse((package / 'config').is_symlink())
            self.assertFalse((package / 'share/config').exists())
            self.assertIsNone(yaml.safe_load((package / 'config/deployment.yaml').read_text())['manifest'])
            self.assertFalse(list((package / 'config/calibration').glob('*/tasks')))
            workspace = base / 'persistent workspace' if external else package
            env = dict(os.environ, T170_PYTHON=sys.executable,
                       T170_STATE_DIR=str(base/'state'), T170_LOG_DIR=str(base/'logs'))
            env.pop('T170_WORKSPACE',None)
            if external: env['T170_WORKSPACE']=str(workspace)
            for key in ('T170_ROOT','T170_RUNTIME_DIR','T170_CONFIG_DIR','T170_DEPLOYMENT','T170_BINARY_ROOT'):
                env.pop(key,None)
            subprocess.run([sys.executable,str(package/'tools/setup_robot.py'),'TH-T170C-0125','--model','3',
                            '--head-serial','test-head','--left-serial','test-left','--right-serial','test-right'],env=env,check=True,capture_output=True)
            camera=yaml.safe_load((workspace/'config/robots/TH-T170C-0125/cameras/realsense_cameras.yaml').read_text())['realsense']
            self.assertEqual(camera['left_hand']['image_rotation_deg'],0)
            self.assertEqual(camera['right_hand']['image_rotation_deg'],180)
            launch = ['bash',str(package/'deploy/run_maintenance.sh')]
            launched=subprocess.run(launch,env=env,capture_output=True,text=True)
            self.assertEqual(launched.returncode,0,launched.stderr+launched.stdout)
            run = next((base/'state').glob('session.*/runtime'))
            self.assertEqual(json.loads((run/'deployment_resolved.json').read_text())['robot_id'],'TH-T170C-0125')
            self.assertEqual(json.loads((run/'deployment_resolved.json').read_text())['resource_root'],str(workspace.resolve()))
            quick = workspace/'config/robots/TH-T170C-0125/quick_params.yaml'
            data = yaml.safe_load(quick.read_text());data['head_grasp']['grasp_z_offset_m']=-.017;quick.write_text(yaml.safe_dump(data))
            subprocess.run([sys.executable,str(package/'tools/apply_config.py'),'--validate-only'],env=env,check=True,capture_output=True)
            subprocess.run(launch,env=env,check=True,capture_output=True)
            runs = list((base/'state').glob('session.*/runtime'))
            self.assertEqual(len(runs),2)
            self.assertIn(-.017,[yaml.safe_load((p/'config/effective_params.yaml').read_text())['head_grasp']['grasp_z_offset_m'] for p in runs])
            self.assertEqual(yaml.safe_load(quick.read_text())['head_grasp']['grasp_z_offset_m'],-.017)
            with socket.socket() as sock:
                sock.bind(('127.0.0.1',0));port=str(sock.getsockname()[1])
            result=subprocess.run([sys.executable,str(package/'tools/upgrade_robot.py'),
                '--robot-port',port,'--web-port',port],env=env,check=True,capture_output=True,text=True)
            self.assertEqual(json.loads(result.stdout)['version'],'layout-test')
            self.assertEqual(yaml.safe_load(quick.read_text())['head_grasp']['grasp_z_offset_m'],-.017)


if __name__ == '__main__': unittest.main()
