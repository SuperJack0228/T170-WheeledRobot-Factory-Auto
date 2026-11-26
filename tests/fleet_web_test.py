"""Real HTTP federation between isolated fake robots; never opens hardware."""
import argparse
import concurrent.futures
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'python')]
from t170_common.fleet import FleetStore, connection_settings
from t170_common.config_workspace import ConfigWorkspace
from web_debug.profiles import ProfileStore
from t170_common.maintenance_workcells import load_catalog


class Agent:
    def __init__(self, root, number):
        self.root = root
        self.robot_id = f'TH-T170C-{number:04d}'
        self.token = f'private-token-{number}'
        self.calls = []
        self.last_motion_target = None
        self.started = threading.Event()
        self.release = threading.Event()
        self.calibration_active = False
        (root / 'config/deployments').mkdir(parents=True)
        directory = root / 'config/robots' / self.robot_id
        directory.mkdir(parents=True)
        (directory / 'identity.yaml').write_text('chassis: {host: 192.168.192.5}\n')
        (root / 'config/deployment.yaml').write_text(f'manifest: deployments/{self.robot_id}.yaml\n')
        (root / 'config/deployments' / (self.robot_id + '.yaml')).write_text(yaml.safe_dump({
            'robot_id': self.robot_id, 'robot_model': 't170c_v2_3kg', 'files': {},
            'calibration_directory': f'config/calibration/{self.robot_id}/tasks'}))
        self.config = root / 'config/example.yaml'
        self.config.write_text(f'owner: {number}\n')
        image = root / 'debug_picture/calibration/test/frame.jpg'
        image.parent.mkdir(parents=True)
        image.write_bytes(b'\xff\xd8' + str(number).encode() + b'\xff\xd9')
        spec = importlib.util.spec_from_file_location(f'fleet_test_agent_{number}', ROOT / 'web_debug/server.py')
        self.web = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.web)
        self.web.PROJECT_ROOT = root
        self.web.RUNTIME_ROOT = root / 'logs/web'
        self.web.WORKSPACE = ConfigWorkspace(root)
        self.web.PROFILE_STORE = ProfileStore(root)
        self.web.FLEET = FleetStore(root)
        self.web.config_path = lambda file_id: self.config
        self.web.host_status = lambda: {}
        self.web.hand_live_preview_enabled = lambda: False
        workcells=load_catalog(ROOT/'config/sites/current_factory/maintenance_workcells.yaml')
        self.web.maintenance_workcells_view=lambda state: dict(workcells,ok=True,editable=True,
            active_group=workcells['selected_group'])
        self.state = self.web.AppState(argparse.Namespace(robot_host='127.0.0.1', robot_port=1,
            robot_command='', token=self.token))
        self.state.robot_request = self.request
        self.state.log(f'owner:{number}')
        self.server = self.web.DebugHTTPServer(('127.0.0.1', 0), self.state)
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def request(self, payload, timeout=30):
        command = payload['cmd']
        self.calls.append(command)
        if command in ('home','dispatch'):
            self.last_motion_target = payload.get('expected_robot_id')
        if command == 'ping':
            return dict(ok=True, robot_id=self.robot_id, robot_model='t170c_v2_3kg',
                mode='maintenance', runtime_root=str(self.root), config_dir=str(self.root/'config'))
        if command == 'dispatch':
            self.started.set()
            self.release.wait(60)
            return dict(ok=False, code=-4)
        if command == 'dispatch_exit':
            self.release.set()
        if command == 'dispatch_status':
            return dict(ok=True, active=self.started.is_set() and not self.release.is_set())
        if command == 'capabilities':
            return dict(ok=True,mode='maintenance',robot_id=self.robot_id,robot_model='t170c_v2_3kg',
                dispatch_active=self.started.is_set() and not self.release.is_set(),
                commands=[dict(name=name,allowed=True,group='operation') for name in
                    ['home','dispatch','dispatch_exit','ping','calibration_status']])
        if command == 'snapshot':
            return dict(ok=True,right_tcp_m_deg=[.4,-.2,-.3,0,55,40],left_tcp_m_deg=[.4,.2,-.3,0,55,-40],
                waist_x=0,waist_y=0,waist_z=.6,waist_joints_deg=[0]*5)
        if command == 'calibration_status':
            return dict(ok=True, active=self.calibration_active, session='test', image_revision=1,runtime_root=str(self.root),
                image_path='debug_picture/calibration/test/frame.jpg')
        return dict(ok=True, code=0, owner=self.robot_id)

    def close(self):
        self.release.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(3)


class FleetHTTPTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='t170-fleet-')
        self.addCleanup(self.tmp.cleanup)
        self.local = Agent(Path(self.tmp.name) / '120', 120)
        self.addCleanup(self.local.close)
        self.remote = Agent(Path(self.tmp.name) / '124', 124)
        self.addCleanup(self.remote.close)
        self.local.web.FLEET.save(dict(robot_id='124', host='127.0.0.1', port=self.remote.port,
            token=self.remote.token))

    def http(self, path, body=None, token=None):
        headers = {'X-T170-Token': token or self.local.token}
        if body is not None:
            headers['Content-Type'] = 'application/json'
        request = urllib.request.Request(f'http://127.0.0.1:{self.local.port}' + path,
            data=json.dumps(body).encode() if body is not None else None, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=12) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()

    def test_remote_identity_parameters_images_and_logs_stay_remote(self):
        prefix = '/robots/TH-T170C-0124'
        code, data = self.http(prefix + '/api/identity')
        self.assertEqual(code, 200)
        self.assertEqual(json.loads(data)['robot_id'], self.remote.robot_id)
        self.assertIn('owner: 124', json.loads(self.http(prefix + '/api/config?file=motion')[1])['content'])
        self.assertIn(b'owner:124', self.http(prefix + '/api/logs?since=0')[1])
        self.remote.calibration_active = True
        self.assertEqual(self.http(prefix + '/api/calibration/image?kind=hand&session=test&revision=1')[1], b'\xff\xd8124\xff\xd9')
        page = self.http(prefix + '/')[1]
        self.assertIn(b'id="fleetRobot"', page)
        self.assertIn(b'src="app.js?v=', page)
        self.assertEqual(self.http(prefix + '/app.js')[0], 200)

    def test_identity_mismatch_and_bad_portal_token_never_send_motion(self):
        self.local.web.FLEET.save(dict(robot_id='125', host='127.0.0.1', port=self.remote.port,
            token=self.remote.token))
        code, data = self.http('/robots/TH-T170C-0125/api/command', {'cmd': 'home'})
        self.assertEqual(code, 502)
        self.assertIn('身份不匹配', data.decode())
        self.assertNotIn('home', self.remote.calls)
        self.assertEqual(self.http('/robots/TH-T170C-0124/api/command',
            {'cmd':'home','expected_robot_id':'TH-T170C-0125'})[0],502)
        self.assertNotIn('home', self.remote.calls)
        code, _ = self.http('/robots/TH-T170C-0124/api/command', {'cmd': 'home'}, token='wrong')
        self.assertEqual(code, 401)
        self.assertNotIn('home', self.remote.calls)

    def test_command_binds_target_and_dispatch_exit_remains_available(self):
        prefix = '/robots/TH-T170C-0124'
        with concurrent.futures.ThreadPoolExecutor() as worker:
            pending = worker.submit(self.http, prefix + '/api/command', {'cmd': 'dispatch', 'confirm': 'dispatch'})
            self.assertTrue(self.remote.started.wait(3))
            self.assertEqual(json.loads(self.http('/api/identity')[1])['robot_id'], self.local.robot_id)
            self.assertTrue(json.loads(self.http(prefix + '/api/dispatch')[1])['active'])
            self.assertEqual(self.http(prefix + '/api/command', {'cmd': 'dispatch_exit'})[0], 200)
            code, data = pending.result(timeout=3)
            self.assertEqual(code, 200)
            self.assertEqual(json.loads(data)['status'], 'stopped')
        self.assertNotIn('dispatch', self.local.calls)
        self.assertEqual(self.remote.last_motion_target,self.remote.robot_id)

    def test_connections_derive_ip_and_never_expose_credentials(self):
        self.assertEqual(connection_settings('120')['host'], '192.168.113.120')
        data = self.http('/api/fleet')[1]
        self.assertNotIn(self.remote.token.encode(), data)
        self.assertEqual(self.local.web.FLEET.secrets.stat().st_mode & 0o777, 0o600)
        self.assertTrue(next(r for r in json.loads(data)['robots'] if r['robot_id']==self.remote.robot_id)['token_saved'])
        code, _ = self.http('/api/fleet', {'robot_id': '126'})
        self.assertEqual(code, 200)
        self.assertEqual(self.local.web.FLEET.connections()['TH-T170C-0126']['host'], '192.168.113.126')

    def test_missing_credentials_do_not_fall_back_to_local_robot(self):
        self.local.web.FLEET.save(dict(robot_id='126'))
        self.assertEqual(self.http('/robots/TH-T170C-0126/api/command', {'cmd':'home'})[0], 502)
        self.assertNotIn('home', self.local.calls)


if __name__ == '__main__':
    unittest.main()
