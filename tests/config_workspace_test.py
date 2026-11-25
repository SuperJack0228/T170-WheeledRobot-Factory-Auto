"""Persistent edits, candidate publication and nominal camera frames; no hardware."""
import copy
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch
from concurrent.futures import ThreadPoolExecutor
import time
import importlib.util
import threading
import urllib.request
import urllib.error
import yaml
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'python')]
CHECKER = sys.argv.pop(1)
from t170_common.config_workspace import ConfigWorkspace, atomic_write
from t170_common.deployment import resolve
from t170_common.camera_nominal import hand_eye_for_servo, SERVO_IMAGES
from t170_common.arm_model import pose_matrix
from web_debug.profiles import ProfileStore


class WorkspaceTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='t170-workspace-'); self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name); self.root = self.base / 'source'
        shutil.copytree(ROOT / 'config', self.root / 'config')
        self.workspace = ConfigWorkspace(self.root, CHECKER)
        self.manifest = self.workspace.manifest()[0]
        self.runtime = self.base / 'live'; resolve(self.root, self.manifest, self.runtime)
        self.code = 'reloaded'; self.requests = []

    def request(self, payload, timeout=30):
        self.requests.append(payload)
        if payload['cmd'] == 'ping': return {'ok': True, 'runtime_root': str(self.runtime)}
        candidate = Path(payload['candidate_config'])
        self.assertEqual(candidate.parents[2], self.runtime)
        if self.code == 'reloaded':
            atomic_write(self.runtime / 'config/effective_params.yaml', candidate.read_text())
            return {'ok': True, 'code': self.code}
        return {'ok': False, 'code': self.code, 'error': self.code}

    def changed(self, value):
        data = yaml.safe_load(self.workspace.source_file('motion').read_text())
        data['head_grasp']['grasp_z_offset_m'] = value
        return yaml.safe_dump(data, sort_keys=False)

    def test_hot_save_is_permanent_after_new_session(self):
        result = self.workspace.save('motion', self.changed(-.012), self.request)
        self.assertTrue(result['applied'])
        next_run = self.base / 'next'; resolve(self.root, self.manifest, next_run)
        self.assertEqual(yaml.safe_load((next_run / 'config/effective_params.yaml').read_text())['head_grasp']['grasp_z_offset_m'], -.012)
        self.assertFalse((next_run / 'config/move_box_params.yaml').exists())
        self.assertFalse(list(self.runtime.glob('.config-apply-*')))

    def test_all_five_groups_resolve_to_the_measured_station_table(self):
        expected={1:['AP1','AP2','AP3','AP4'],2:['AP6','AP9','AP8','AP10'],
                  3:['AP407','AP408','AP409','AP410'],4:['AP413','AP414','AP415','AP416'],
                  5:['AP418','AP419','AP420','AP421']}
        path=self.workspace.source_file('maintenance_workcells')
        data=yaml.safe_load(path.read_text())
        fields=('tray_station','belt_station','out_station','tray2_station')
        for group,stations in expected.items():
            data['selected_group']=group
            result=self.workspace.save('maintenance_workcells',yaml.safe_dump(data),self.request)
            self.assertTrue(result['applied'])
            effective=yaml.safe_load((self.runtime/'config/effective_params.yaml').read_text())
            self.assertEqual([effective['chassis'][key] for key in fields],stations)
            record=json.loads((self.runtime/'deployment_resolved.json').read_text())
            self.assertEqual(record['task_assignment'],'dispatch')
            for key in fields:self.assertEqual(record['parameter_sources']['chassis.'+key],
                'config/sites/current_factory/maintenance_workcells.yaml')
        quick=yaml.safe_load(self.workspace.source_file('motion').read_text())
        self.assertTrue(set(fields).isdisjoint(quick['chassis']))

    def test_invalid_quick_file_rolls_back_persistent_and_live_files(self):
        path = self.workspace.source_file('motion'); previous = path.read_bytes()
        snapshot = (self.runtime / 'config/effective_params.yaml').read_bytes()
        with self.assertRaises(ValueError): self.workspace.save('motion', 'unsupported: 1\n', self.request)
        self.assertEqual(path.read_bytes(), previous)
        self.assertEqual((self.runtime / 'config/effective_params.yaml').read_bytes(), snapshot)

    def test_busy_or_restart_keeps_saved_value_for_next_boot(self):
        for code in ('busy', 'restart_required', 'calibration_active'):
            self.code = code
            result = self.workspace.save('motion', self.changed(-.013), self.request)
            self.assertTrue(result['saved']); self.assertFalse(result['applied'])
            self.assertEqual(yaml.safe_load(self.workspace.source_file('motion').read_text())['head_grasp']['grasp_z_offset_m'], -.013)

    def test_offline_apply_validates_any_selected_yaml(self):
        defaults = self.root / 'config/defaults/factory_baseline.yaml'
        data = yaml.safe_load(defaults.read_text()); data['visual_servo']['kp'] = 1.7
        defaults.write_text(yaml.safe_dump(data)); self.assertEqual(self.workspace.apply()['code'], 'validated_offline')
        data['visual_servo']['kp'] = -1; defaults.write_text(yaml.safe_dump(data))
        with self.assertRaises(ValueError): self.workspace.apply()

    def test_head_editor_uses_actual_theory_file(self):
        self.assertEqual(self.workspace.source_file('hand_eye').name, 'extrinsics_urdf.yaml')

    def test_legacy_measured_eye_follows_declared_old_image_axes(self):
        binding=yaml.safe_load((self.runtime/'config/arm_binding.yaml').read_text())
        frames=yaml.safe_load((self.runtime/'config/arm_frames.yaml').read_text())
        eye=yaml.safe_load((self.root/'config/calibration/TH-T170C-0124/hand_eye/left.yaml').read_text())
        old=hand_eye_for_servo(eye,binding,frames,'left',180)
        new=hand_eye_for_servo(eye,binding,frames,'left',0)
        np.testing.assert_allclose(np.array(new['a. matrix']),np.array(old['a. matrix'])@np.diag([-1,-1,1,1]))

    def test_metadata_failure_after_commit_does_not_revert_parameters(self):
        real_write = atomic_write
        def fail_record(path, content):
            if Path(path) == self.runtime / 'deployment_resolved.json':
                raise OSError('injected record write failure')
            real_write(path, content)
        with patch('t170_common.config_workspace.atomic_write', side_effect=fail_record):
            result = self.workspace.save('motion', self.changed(-.014), self.request)
        self.assertTrue(result['applied']); self.assertEqual(result['code'], 'reloaded_metadata_pending')
        self.assertEqual(yaml.safe_load(self.workspace.source_file('motion').read_text())['head_grasp']['grasp_z_offset_m'], -.014)
        self.assertEqual(yaml.safe_load((self.runtime / 'config/effective_params.yaml').read_text())['head_grasp']['grasp_z_offset_m'], -.014)
        self.assertTrue(list(self.runtime.glob('.config-apply-*')))

    def test_lost_commit_reply_keeps_persistent_source_and_candidate(self):
        def lost_reply(payload, timeout=30):
            result = self.request(payload, timeout)
            if payload['cmd'] == 'reload_config': raise ConnectionError('injected lost acknowledgement')
            return result
        result = self.workspace.save('motion', self.changed(-.015), lost_reply)
        self.assertIsNone(result['applied']); self.assertEqual(result['code'], 'apply_unconfirmed')
        self.assertEqual(yaml.safe_load(self.workspace.source_file('motion').read_text())['head_grasp']['grasp_z_offset_m'], -.015)
        self.assertTrue(list(self.runtime.glob('.config-apply-*')))

    def test_concurrent_field_edits_preserve_other_changes(self):
        def edit(field, value):
            def render(text):
                data = yaml.safe_load(text)
                time.sleep(.03)
                data['head_grasp'][field] = value
                return yaml.safe_dump(data)
            return ConfigWorkspace(self.root, CHECKER).edit('motion', render)
        with ThreadPoolExecutor(max_workers=2) as workers:
            futures = [workers.submit(edit, 'grasp_z_offset_m', -.019),
                       workers.submit(edit, 'right_grasp_z_offset_m', -.018)]
            for future in futures: self.assertTrue(future.result()['saved'])
        data = yaml.safe_load(self.workspace.source_file('motion').read_text())['head_grasp']
        self.assertEqual(data['grasp_z_offset_m'], -.019)
        self.assertEqual(data['right_grasp_z_offset_m'], -.018)

    def test_new_robot_defaults_to_nominal_camera_frames_for_both_models(self):
        store = ProfileStore(self.root)
        for model, robot in [('3', 'TH-T170C-0125'), ('5', 'TH-T170C-0126')]:
            result = store.update({'action': 'create', 'expected_manifest': store.selected(),
                'template': store.selected(), 'robot_id': robot, 'robot_model': 't170c_v2_'+model+'kg', 'host': '192.168.192.6'})
            run = self.base / robot; record = resolve(self.root, self.root / result['manifest'], run)
            self.assertEqual(len(record['missing_task_calibrations']), 4)
            sdk = yaml.safe_load((run / 'config/Robot_Model.yaml').read_text())['T7_170']
            controls = yaml.safe_load((run / 'config/effective_params.yaml').read_text())
            for side in ('left', 'right'):
                source = yaml.safe_load((self.root / ('config/robot_models/t170c_v2_common/cameras/'+side+'_nominal.yaml')).read_text())
                actual = yaml.safe_load((run / ('config/'+side+'_hand_to_base_result.yaml')).read_text())
                eye = np.array(actual['a. matrix'], dtype=float); eye[:3,3] *= .001
                nominal = np.array(source['a. matrix'], dtype=float); nominal[:3,3] *= .001
                # Independent physical invariant: model/frame selection must not
                # change camera relative to TCP, just as for verified No.124.
                expected = np.linalg.inv(np.array(pose_matrix([.26,0,0,0,0,0]))) @ nominal
                tcp_camera = np.linalg.inv(np.array(pose_matrix(sdk[side]['tool']))) @ eye
                np.testing.assert_allclose(tcp_camera, expected, rtol=0, atol=1e-12)
                self.assertEqual(actual['source_parent_frame'], 'legacy_t7_flange')
                self.assertEqual(actual['image_convention'], SERVO_IMAGES[side])
                # All deployed grasp rows and placement attitudes must look down.
                attitudes = controls['head_grasp']['row_grasp_rpy_deg'][side]
                attitudes = attitudes + [controls['tray2_place']['rpy_deg'][side]]
                for attitude in attitudes:
                    rotation = np.array(pose_matrix([0,0,0,*np.deg2rad(attitude)]))[:3,:3]
                    self.assertLess((rotation @ tcp_camera[:3,:3])[2,2], -.8)

                # No.120 field reference: a valid ray in front of the camera.
                if side == 'left':
                    pose = [.48917521546373061,.093176383946879951,-.40726104702148724,
                            .080424079262245304,.95963661157112246,-.87145646508166719]
                    camera = np.array(pose_matrix(pose)) @ tcp_camera
                    ray = np.array([(718.85862516212705-643.52227783203125)/656.799560546875,
                                    (462.80770809185265-366.93551635742188)/656.2257080078125,1.])
                    depth = (-.41645478616154102-camera[2,3])/(camera[:3,:3] @ ray)[2]
                    self.assertAlmostEqual(depth, .11608143755036156, places=10)


class PersistentHTTPTest(unittest.TestCase):
    def setUp(self):
        fixture=WorkspaceTest('test_hot_save_is_permanent_after_new_session');fixture.setUp()
        self.addCleanup(fixture.doCleanups);self.fixture=fixture
        spec=importlib.util.spec_from_file_location('persistent_config_web',ROOT/'web_debug/server.py')
        self.web=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.web)
        self.web.WORKSPACE=fixture.workspace
        class State:
            token='test'
            mode='maintenance'
            running=False
            def begin_action(self, title):
                if self.running:return False
                self.running=True;return True
            def end_action(self, error=''):self.running=False
            def robot_request(_,payload,timeout=30):
                if payload['cmd']=='calibration_status':return {'ok':True,'active':False}
                result=fixture.request(payload,timeout)
                if payload['cmd']=='ping':result.update(mode=_.mode,config_dir=str(fixture.runtime/'config'))
                return result
            def log(_, *args):pass
        self.server=self.web.DebugHTTPServer(('127.0.0.1',0),State())
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.close_server)

    def workcell_request(self, group=None):
        url=f'http://127.0.0.1:{self.server.server_port}/api/maintenance-workcells'
        data=None if group is None else json.dumps({'group':group}).encode()
        request=urllib.request.Request(url,data=data,headers={'X-T170-Token':'test','Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(request,timeout=10) as reply:return reply.status,json.load(reply)
        except urllib.error.HTTPError as error:return error.code,json.load(error)

    def test_workcell_selection_updates_live_and_persistent_station_defaults(self):
        status,before=self.workcell_request()
        self.assertEqual(status,200);self.assertEqual(before['active_group'],1)
        status,result=self.workcell_request(2)
        self.assertEqual(status,200,result);self.assertTrue(result['applied'])
        self.assertEqual(result['workcells']['active_group'],2)
        actual=yaml.safe_load((self.fixture.runtime/'config/effective_params.yaml').read_text())['chassis']
        self.assertEqual([actual[key] for key in ('tray_station','belt_station','out_station','tray2_station')],
                         ['AP6','AP9','AP8','AP10'])
        run=self.fixture.base/'next-session';resolve(self.fixture.root,self.fixture.manifest,run)
        self.assertEqual(yaml.safe_load((run/'config/effective_params.yaml').read_text())['chassis']['out_station'],'AP8')
        self.assertFalse(any(item['cmd'] not in ('ping','reload_config') for item in self.fixture.requests))

    def test_workcell_change_is_rejected_and_rolled_back_while_busy(self):
        self.fixture.code='busy'
        path=self.fixture.workspace.source_file('maintenance_workcells');before=path.read_bytes()
        status,result=self.workcell_request(2)
        self.assertEqual(status,400,result);self.assertEqual(path.read_bytes(),before)
        self.assertEqual(self.workcell_request()[1]['active_group'],1)

    def test_workcell_change_is_rejected_in_production_and_for_unknown_groups(self):
        path=self.fixture.workspace.source_file('maintenance_workcells');before=path.read_bytes()
        self.server.app_state.mode='production'
        self.assertEqual(self.workcell_request(2)[0],400)
        self.assertEqual(path.read_bytes(),before)
        self.server.app_state.mode='maintenance'
        self.assertEqual(self.workcell_request(99)[0],400)
        self.assertEqual(path.read_bytes(),before)

    def close_server(self):
        self.server.shutdown();self.server.server_close();self.thread.join()

    def save_fields(self,values):
        request=urllib.request.Request(f'http://127.0.0.1:{self.server.server_port}/api/fields',
            data=json.dumps({'file':'motion','values':values,'confirm':'save_config'}).encode(),
            headers={'X-T170-Token':'test','Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(request,timeout=5) as reply:return reply.status,json.load(reply)
        except urllib.error.HTTPError as error:return error.code,json.load(error)

    def test_web_save_uses_persistent_file_and_shared_apply(self):
        status,result=self.save_fields({'head_grasp.grasp_z_offset_m':'-0.021'})
        self.assertEqual(status,200);self.assertTrue(result['applied'])
        self.assertEqual(yaml.safe_load(self.fixture.workspace.source_file('motion').read_text())['head_grasp']['grasp_z_offset_m'],-.021)

    def test_save_all_quick_fields_with_block_arrays_and_restart(self):
        fields={field['key']:str(field['value']) for group in self.web.read_field_groups('motion') for field in group['fields']}
        self.assertEqual(len(fields),94)
        fields['waist.layer3_home.2']='0.61'
        fields['head_grasp.row1_rpy_deg.left.1']='42'
        status,result=self.save_fields(fields)
        self.assertEqual(status,200,result);self.assertTrue(result['applied'])
        run=self.fixture.base/'after-web-save';resolve(self.fixture.root,self.fixture.manifest,run)
        data=yaml.safe_load((run/'config/effective_params.yaml').read_text())
        self.assertEqual(data['waist']['layer3_home'][2],.61)
        self.assertEqual(data['head_grasp']['row1_rpy_deg']['left'],[0,42,-30])

    def test_scalar_spans_preserve_comments_and_support_flow_and_block(self):
        for text in ['a: [1, 2, 3] # keep\n','a:\n  - 1 # first\n  - 2 # second\n  - 3\n']:
            result=self.web._replace_yaml_value(text,'a.1','4.5')
            self.assertEqual(yaml.safe_load(result),{'a':[1,4.5,3]})
            self.assertEqual(text.count('#'),result.count('#'))
        text='a: {x: 1, y: 2} # keep\n'
        self.assertEqual(self.web._replace_yaml_value(text,'a.x','3'),'a: {x: 3, y: 2} # keep\n')
        text=yaml.safe_dump({'a. matrix':[[1,0,0,1],[0,1,0,2],[0,0,1,3],[0,0,0,1]],
                            'b. translation_xyz_mm':[1,2,3]},sort_keys=False)
        data=yaml.safe_load(self.web._sync_hand_eye_matrix(text,[4,5,6]))
        self.assertEqual([row[3] for row in data['a. matrix']],[4,5,6,1])

    def test_web_invalid_value_rolls_back(self):
        path=self.fixture.workspace.source_file('motion');before=path.read_bytes()
        status,_=self.save_fields({'waist.layer3_home.2':'not-a-number'})
        self.assertEqual(status,400);self.assertEqual(path.read_bytes(),before)

    def test_web_busy_reports_saved_but_not_applied(self):
        self.fixture.code='busy'
        status,result=self.save_fields({'head_grasp.grasp_z_offset_m':'-0.022'})
        self.assertEqual(status,200);self.assertTrue(result['saved']);self.assertFalse(result['applied'])
        self.assertEqual(result['code'],'busy')


if __name__ == '__main__': unittest.main()
