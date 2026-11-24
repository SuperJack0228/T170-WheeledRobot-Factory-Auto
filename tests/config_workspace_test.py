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
            frames = yaml.safe_load((run / 'config/arm_frames.yaml').read_text())
            for side in ('left', 'right'):
                source = yaml.safe_load((self.root / ('config/robot_models/t170c_v2_common/cameras/'+side+'_nominal.yaml')).read_text())
                actual = yaml.safe_load((run / ('config/'+side+'_hand_to_base_result.yaml')).read_text())
                expected = np.array(frames[side]['sdk_flange_from_urdf_wrist']); expected[:3,3] *= 1000
                expected = expected @ np.array(source['a. matrix']) @ np.diag([-1,-1,1,1])
                np.testing.assert_allclose(actual['a. matrix'], expected, atol=1e-10)
                self.assertEqual(actual['image_convention'], SERVO_IMAGES[side])
                # Rotation changes only axes; camera centre stays in the same place.
                np.testing.assert_allclose(np.array(actual['a. matrix'])[:3,3], expected[:3,3], atol=1e-10)


class PersistentHTTPTest(unittest.TestCase):
    def setUp(self):
        fixture=WorkspaceTest('test_hot_save_is_permanent_after_new_session');fixture.setUp()
        self.addCleanup(fixture.doCleanups);self.fixture=fixture
        spec=importlib.util.spec_from_file_location('persistent_config_web',ROOT/'web_debug/server.py')
        self.web=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.web)
        self.web.WORKSPACE=fixture.workspace
        class State:
            token='test'
            def robot_request(_,payload,timeout=30):
                if payload['cmd']=='calibration_status':return {'ok':True,'active':False}
                return fixture.request(payload,timeout)
            def log(_, *args):pass
        self.server=self.web.DebugHTTPServer(('127.0.0.1',0),State())
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.close_server)

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
