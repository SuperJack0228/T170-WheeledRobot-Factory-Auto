"""Upgrade the real persistent config shape with a real C++ checker, no hardware."""
from pathlib import Path
import importlib.util
import json
import shutil
import sys
import tempfile
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]
CHECKER=Path(sys.argv.pop(1)).resolve()
sys.path[:0]=[str(ROOT),str(ROOT/'python')]
from web_debug.profiles import ProfileStore
from t170_common.installation import initialize
from t170_common.deployment import resolve
spec=importlib.util.spec_from_file_location('upgrade_robot',ROOT/'tools/upgrade_robot.py')
upgrade_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(upgrade_module)


class UpgradeTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='t170-upgrade-');self.addCleanup(self.temp.cleanup)
        self.base=Path(self.temp.name);self.root=self.base/'workspace';self.root.mkdir()
        shutil.copytree(ROOT/'config',self.root/'config')
        self.old=self.package('old','1.0.0-rc.1');self.new=self.package('new','1.0.0-rc.2')
        initialize(self.old,self.root)

    def package(self,name,version):
        path=self.base/name;path.mkdir();(path/'bin').mkdir();(path/'lib').mkdir()
        (path/'bin/t170_config_dump').symlink_to(CHECKER)
        (path/'release.json').write_text(json.dumps({'version':version}))
        (path/'environment.lock.json').write_text('{}')
        return path

    def test_no124_legacy_cameras_and_references_are_preserved(self):
        camera=self.root/'config/robots/TH-T170C-0124/cameras/realsense_cameras.yaml'
        data=yaml.safe_load(camera.read_text())
        for side in ('left','right'):data['realsense'][side+'_hand'].pop('image_rotation_deg')
        camera.write_text(yaml.safe_dump(data))
        tasks=self.root/'config/calibration/TH-T170C-0124/tasks'
        before={p.relative_to(tasks):p.read_bytes() for p in tasks.glob('*/*.yaml')}
        result=upgrade_module.upgrade(self.new,self.root,self.base/'backups')
        self.assertFalse(result['camera_orientation_changed']);self.assertFalse(result['retired_task_references'])
        data=yaml.safe_load(camera.read_text())
        self.assertEqual(data['realsense']['left_hand']['image_rotation_deg'],180)
        self.assertEqual(data['realsense']['right_hand']['image_rotation_deg'],0)
        self.assertEqual(before,{p.relative_to(tasks):p.read_bytes() for p in tasks.glob('*/*.yaml')})
        self.assertEqual((self.root/'bin').resolve(),(self.new/'bin').resolve())

    def test_no120_retains_parameters_and_archives_only_changed_references(self):
        store=ProfileStore(self.root)
        created=store.update({'action':'create','expected_manifest':store.selected(),'template':store.selected(),
            'robot_id':'TH-T170C-0120','robot_model':'t170c_v2_3kg','host':'192.168.192.5'})
        store.update({'action':'select','expected_manifest':store.selected(),'manifest':created['manifest'],'robot_model':'t170c_v2_3kg'})
        camera=self.root/'config/robots/TH-T170C-0120/cameras/realsense_cameras.yaml'
        data=yaml.safe_load(camera.read_text())
        for side in ('left','right'):data['realsense'][side+'_hand'].pop('image_rotation_deg')
        camera.write_text(yaml.safe_dump(data))
        quick=self.root/'config/robots/TH-T170C-0120/quick_params.yaml'
        data=yaml.safe_load(quick.read_text());data['head_grasp']['grasp_z_offset_m']=-.023
        quick.write_text(yaml.safe_dump(data));before=quick.read_bytes()
        pixels=self.root/'config/robots/TH-T170C-0120/reference_pixels.yaml'
        shutil.copyfile(ROOT/'config/robots/TH-T170C-0124/reference_pixels.yaml',pixels)
        tasks=self.root/'config/calibration/TH-T170C-0120/tasks'
        for task in ('grasp','placement'):
            (tasks/task).mkdir()
            for side in ('left','right'):(tasks/task/(side+'.yaml')).write_text('old task reference\n')
        result=upgrade_module.upgrade(self.new,self.root,self.base/'backups')
        self.assertEqual(result['camera_orientation_changed'],['TH-T170C-0120'])
        self.assertEqual(len(result['retired_task_references']),4)
        self.assertEqual(quick.read_bytes(),before)
        self.assertEqual(len(list(tasks.glob('orientation_before_*/*/*.yaml'))),4)
        self.assertFalse(list(tasks.glob('grasp/*.yaml')))
        self.assertTrue(Path(result['backup'],'config/calibration/TH-T170C-0120/tasks/grasp/left.yaml').exists())
        data=yaml.safe_load(pixels.read_text());self.assertEqual(data['cameras']['left_gripper_u'],1279-588)
        second=upgrade_module.upgrade(self.new,self.root,self.base/'backups')
        self.assertFalse(second['camera_orientation_changed'])
        self.assertEqual(quick.read_bytes(),before)

    def test_validation_failure_restores_config_and_old_program(self):
        quick=self.root/'config/robots/TH-T170C-0124/quick_params.yaml'
        data=yaml.safe_load(quick.read_text());data['head_grasp']['descend_vel_m_s']=-1
        quick.write_text(yaml.safe_dump(data));before=quick.read_bytes()
        with self.assertRaises(ValueError):upgrade_module.upgrade(self.new,self.root,self.base/'backups')
        self.assertEqual(quick.read_bytes(),before)
        self.assertEqual((self.root/'bin').resolve(),(self.old/'bin').resolve())
        self.assertEqual(json.loads((self.root/'release.json').read_text())['version'],'1.0.0-rc.1')


if __name__=='__main__':unittest.main()
