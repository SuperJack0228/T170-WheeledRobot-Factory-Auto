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
        catalog=path/'share/config/sites/current_factory/conveyor_heights.yaml';catalog.parent.mkdir(parents=True)
        shutil.copy2(ROOT/'config/sites/current_factory/conveyor_heights.yaml',catalog)
        shutil.copy2(ROOT/'config/sites/current_factory/maintenance_workcells.yaml',catalog.parent/'maintenance_workcells.yaml')
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
        receipt=self.root/'config/robots/TH-T170C-0124/parameter_migrations.yaml'
        receipt.unlink()
        quick=self.root/'config/robots/TH-T170C-0124/quick_params.yaml'
        data=yaml.safe_load(quick.read_text());data['head_grasp']['descend_vel_m_s']=-1
        data['head_grasp']['goal_x_offset']=.04
        quick.write_text(yaml.safe_dump(data));before=quick.read_bytes()
        with self.assertRaises(ValueError):upgrade_module.upgrade(self.new,self.root,self.base/'backups')
        self.assertEqual(quick.read_bytes(),before)
        self.assertFalse(receipt.exists())
        self.assertEqual((self.root/'bin').resolve(),(self.old/'bin').resolve())
        self.assertEqual(json.loads((self.root/'release.json').read_text())['version'],'1.0.0-rc.1')

    def test_upgrade_adds_missing_site_catalog_without_resetting_belt_geometry(self):
        catalog=self.root/'config/sites/current_factory/conveyor_heights.yaml';catalog.unlink()
        manifest=self.root/'config/deployments/TH-T170C-0124.yaml'
        doc=yaml.safe_load(manifest.read_text());doc['files'].pop('config/conveyor_heights.yaml')
        manifest.write_text(yaml.safe_dump(doc))
        belt=self.root/'config/sites/current_factory/vision/belt.yaml'
        data=yaml.safe_load(belt.read_text());data['belt'].pop('height_catalog_file');data['belt']['pitch_y_m']=.111
        belt.write_text(yaml.safe_dump(data))
        result=upgrade_module.upgrade(self.new,self.root,self.base/'backups')
        self.assertTrue(result['saved']);self.assertTrue(catalog.is_file())
        data=yaml.safe_load(belt.read_text())
        self.assertEqual(data['belt']['pitch_y_m'],.111)
        self.assertEqual(data['belt']['height_catalog_file'],'../conveyor_heights.yaml')

    def prepare_local_package(self, package):
        shutil.copytree(ROOT/'config',package/'config')
        (package/'config/deployment.yaml').write_text('manifest: null\n')

    def test_import_old_workspace_then_upgrade_between_package_directories(self):
        self.prepare_local_package(self.new)
        quick=self.root/'config/robots/TH-T170C-0124/quick_params.yaml'
        data=yaml.safe_load(quick.read_text());data['head_grasp']['grasp_z_offset_m']=-.023
        quick.write_text(yaml.safe_dump(data));before=quick.read_bytes()
        source_before={str(p.relative_to(self.root/'config')):p.read_bytes()
                       for p in (self.root/'config').rglob('*') if p.is_file()}
        result=upgrade_module.upgrade(self.new,self.new,self.base/'backups',source=self.root)
        self.assertEqual(result['workspace'],str(self.new.resolve()))
        self.assertEqual((self.new/quick.relative_to(self.root)).read_bytes(),before)
        self.assertFalse((self.new/'config').is_symlink())
        self.assertEqual((self.root/'bin').resolve(),(self.old/'bin').resolve())
        self.assertFalse((self.new/'bin').is_symlink())
        self.assertEqual(source_before,{str(p.relative_to(self.root/'config')):p.read_bytes()
                                      for p in (self.root/'config').rglob('*') if p.is_file()})
        latest=self.package('latest','1.0.0-rc.4');self.prepare_local_package(latest)
        second=upgrade_module.upgrade(latest,latest,self.base/'backups',source=self.new)
        self.assertEqual(second['previous_version'],'1.0.0-rc.2')
        self.assertEqual((latest/quick.relative_to(self.root)).read_bytes(),before)
        self.assertFalse(second['retired_task_references'])

    def test_failed_import_restores_new_package_and_keeps_source(self):
        self.prepare_local_package(self.new)
        quick=self.root/'config/robots/TH-T170C-0124/quick_params.yaml'
        data=yaml.safe_load(quick.read_text());data['head_grasp']['descend_vel_m_s']=-1
        quick.write_text(yaml.safe_dump(data));before=quick.read_bytes()
        original=(self.new/quick.relative_to(self.root)).read_bytes()
        with self.assertRaises(ValueError):
            upgrade_module.upgrade(self.new,self.new,self.base/'backups',source=self.root)
        self.assertEqual(quick.read_bytes(),before)
        self.assertEqual((self.new/quick.relative_to(self.root)).read_bytes(),original)
        self.assertIsNone(yaml.safe_load((self.new/'config/deployment.yaml').read_text())['manifest'])
        self.assertFalse((self.new/'bin').is_symlink())
        self.assertEqual((self.root/'bin').resolve(),(self.old/'bin').resolve())

    def test_fresh_package_requires_explicit_import_source(self):
        self.prepare_local_package(self.new)
        with self.assertRaisesRegex(ValueError,'--from-workspace'):
            upgrade_module.upgrade(self.new,self.new,self.base/'backups')

    def test_rc3_no120_offsets_migrate_once_without_losing_other_settings(self):
        store=ProfileStore(self.root)
        created=store.update({'action':'create','expected_manifest':store.selected(),'template':store.selected(),
            'robot_id':'TH-T170C-0120','robot_model':'t170c_v2_3kg','host':'192.168.192.5'})
        store.update({'action':'select','expected_manifest':store.selected(),'manifest':created['manifest'],'robot_model':'t170c_v2_3kg'})
        robot=self.root/'config/robots/TH-T170C-0120'
        (robot/'parameter_migrations.yaml').unlink()
        (self.old/'release.json').write_text(json.dumps({'version':'1.0.0-rc.3'}))
        quick=robot/'quick_params.yaml';data=yaml.safe_load(quick.read_text())
        data['tray']['z_ref_m']=-.618
        data['head_grasp'].update(goal_x_offset=.04,goal_y_offset=.01,grasp_z_offset_m=-.01)
        data['head_grasp']['descend_vel_m_s']=.083
        data['conveyor2']['grasp_offset']['right']={'x':-.03,'y':.135,'z':-.016}
        quick.write_text(yaml.safe_dump(data));before=quick.read_bytes()
        layer=self.root/'config/sites/current_factory/tray_and_navigation.yaml'
        site=yaml.safe_load(layer.read_text());site['head_grasp']['use_hand_camera']=True
        site['head_grasp']['hand_grasp']={'left':{'offset_x':.02,'offset_y':.01}}
        layer.write_text(yaml.safe_dump(site))
        workcells=self.root/'config/sites/current_factory/workcell_profiles.yaml'
        site=yaml.safe_load(workcells.read_text());site['conveyor2']['prefer_belt']='belt0'
        workcells.write_text(yaml.safe_dump(site))
        pixels=(robot/'reference_pixels.yaml').read_bytes()
        result=upgrade_module.upgrade(self.new,self.root,self.base/'backups')
        self.assertEqual(result['task_offsets_migrated'],['TH-T170C-0120'])
        self.assertEqual((Path(result['backup'])/'config/robots/TH-T170C-0120/quick_params.yaml').read_bytes(),before)
        data=yaml.safe_load(quick.read_text())
        self.assertEqual(data['tray']['z_ref_m'],-.623)
        for key in ('goal_x_offset','goal_y_offset','grasp_z_offset_m',
                    'right_goal_x_offset','right_goal_y_offset','right_grasp_z_offset_m'):
            self.assertEqual(data['head_grasp'][key],0)
        for side in ('left','right'):
            self.assertEqual(data['tray2_place']['offset'][side],dict(x=0.,y=0.,z=0.))
        for belt in ('conveyor','conveyor2'):
            self.assertEqual(data[belt]['grasp_offset']['left'],dict(x=.05,y=.12,z=0.))
            self.assertEqual(data[belt]['grasp_offset']['right'],dict(x=-.03,y=.12,z=0.))
            self.assertEqual(data[belt]['offset']['left'],dict(x=-.07,y=-.015,z=0.))
            self.assertEqual(data[belt]['offset']['right'],dict(x=0.,y=-.015,z=0.))
        self.assertEqual(data['head_grasp']['descend_vel_m_s'],.083)
        self.assertEqual((robot/'reference_pixels.yaml').read_bytes(),pixels)
        self.assertNotIn('use_hand_camera',yaml.safe_load(layer.read_text())['head_grasp'])
        self.assertEqual(yaml.safe_load(workcells.read_text())['conveyor2']['prefer_belt'],'belt1')
        # Later field tuning must survive subsequent upgrades and height must
        # never be decremented by another 5 mm on every import.
        data['tray']['z_ref_m']=-.624
        data['head_grasp']['grasp_z_offset_m']=.002
        quick.write_text(yaml.safe_dump(data));tuned=quick.read_bytes()
        second=upgrade_module.upgrade(self.new,self.root,self.base/'backups')
        self.assertFalse(second['task_offsets_migrated'])
        self.assertEqual(quick.read_bytes(),tuned)

    def test_upgrade_moves_old_station_fields_to_site_catalog(self):
        catalog=self.root/'config/sites/current_factory/maintenance_workcells.yaml'
        catalog.unlink()
        manifest=self.root/'config/deployments/TH-T170C-0124.yaml'
        doc=yaml.safe_load(manifest.read_text());doc['files'].pop('config/maintenance_workcells.yaml')
        manifest.write_text(yaml.safe_dump(doc))
        quick=self.root/'config/robots/TH-T170C-0124/quick_params.yaml'
        data=yaml.safe_load(quick.read_text())
        data['chassis'].update(tray_station='AP1',belt_station='AP2',out_station='AP3',tray2_station='AP4')
        quick.write_text(yaml.safe_dump(data))
        result=upgrade_module.upgrade(self.new,self.root,self.base/'backups')
        self.assertTrue(result['saved']);self.assertTrue(catalog.is_file())
        self.assertNotIn('out_station',yaml.safe_load(quick.read_text())['chassis'])
        data=yaml.safe_load(catalog.read_text());data['selected_group']=2
        catalog.write_text(yaml.safe_dump(data))
        run=self.base/'group2';resolve(self.root,manifest,run)
        self.assertEqual(yaml.safe_load((run/'config/effective_params.yaml').read_text())['chassis']['out_station'],'AP8')

    def test_right_place_y_correction_does_not_reset_previous_field_tuning(self):
        from t170_common.task_offset_migration import RIGHT_PLACE_Y_REVISION
        receipt=self.root/'config/robots/TH-T170C-0124/parameter_migrations.yaml'
        data=yaml.safe_load(receipt.read_text());data['applied'].remove(RIGHT_PLACE_Y_REVISION)
        receipt.write_text(yaml.safe_dump(data))
        quick=self.root/'config/robots/TH-T170C-0124/quick_params.yaml'
        data=yaml.safe_load(quick.read_text());data['tray']['z_ref_m']=-.625
        for belt in ('conveyor','conveyor2'):
            data[belt]['offset']['right']['y']=.015
            data[belt]['grasp_offset']['left']['x']=.051
        quick.write_text(yaml.safe_dump(data))
        upgrade_module.upgrade(self.new,self.root,self.base/'backups')
        data=yaml.safe_load(quick.read_text())
        self.assertEqual(data['tray']['z_ref_m'],-.625)
        for belt in ('conveyor','conveyor2'):
            self.assertEqual(data[belt]['offset']['right']['y'],-.015)
            self.assertEqual(data[belt]['grasp_offset']['left']['x'],.051)


if __name__=='__main__':unittest.main()
