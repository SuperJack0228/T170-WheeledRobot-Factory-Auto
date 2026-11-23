"""Real HTTP endpoints with a fake robot transport. Does not open CAN/cameras."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
import unittest
import urllib.request
import urllib.error
import types
import tempfile

ROOT = Path(__file__).resolve().parents[1]
# These routing tests do not parse YAML; a sentinel also permits stock Python.
try:
    import yaml
except ImportError:
    sys.modules['yaml'] = types.ModuleType('yaml')
spec = importlib.util.spec_from_file_location('calibration_web_server', ROOT/'web_debug/server.py')
web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web)

class FakeState:
    token = 'offline-test'
    def __init__(self):
        self.requests = []
        self.running = False
        self.active = False
        self.runtime = None
        self.mode = 'maintenance'
    def robot_request(self, payload, timeout=None):
        self.requests.append(dict(payload))
        if payload['cmd'] == 'calibration_status':
            return {'ok':True,'active':self.active,'session':'session-1','image_revision':3,
                    'image_path':'debug_picture/calibration/session-1/frame_3.jpg' if self.runtime else '../../etc/passwd',
                    'head_image_path':'debug_picture/calibration/session-1/head.jpg',
                    'head_image_revision':2,'runtime_root':str(self.runtime) if self.runtime else ''}
        if payload['cmd'] == 'ping':
            return {'ok':True,'mode':self.mode,'runtime_root':str(self.runtime) if self.runtime else ''}
        if payload['cmd'] == 'capabilities':
            return {'ok':True,'mode':self.mode,'commands':[
                {'name':'home','allowed':self.mode=='maintenance','group':'maintenance'},
                {'name':'tray_servo_test','allowed':False,'group':'retired'}]}
        return {'ok':True,'session':'session-1','phase':'prepared'}
    def begin_action(self, title):
        if self.running:return False
        self.running=True
        return True
    def end_action(self,error=""):self.running=False
    def log(self, *args):pass

class CalibrationHTTPTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.state = FakeState()
        cls.server = web.DebugHTTPServer(('127.0.0.1',0),cls.state)
        cls.thread = threading.Thread(target=cls.server.serve_forever,daemon=True)
        cls.thread.start()
    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown();cls.server.server_close();cls.thread.join()
    def request(self,path,body=None,token=True):
        headers={'X-T170-Token':self.state.token} if token else {}
        if body is not None:headers['Content-Type']='application/json'
        req=urllib.request.Request(f'http://127.0.0.1:{self.server.server_port}'+path,
            data=json.dumps(body).encode() if body is not None else None,headers=headers)
        try:
            with urllib.request.urlopen(req,timeout=3) as r:return r.status,r.read()
        except urllib.error.HTTPError as e:return e.code,e.read()
    def setUp(self):
        self.state.requests.clear();self.state.running=False;self.state.active=False
        self.state.runtime=None;self.state.mode='maintenance'
    def test_deployment_generated_config_cannot_be_overwritten(self):
        from unittest.mock import patch
        with patch.dict(web.os.environ, {'T170_DEPLOYMENT':'selected-profile'}):
            self.assertEqual(self.request('/api/config', {'file':'motion','content':'bad','confirm':'save_config'})[0],409)
            self.assertEqual(self.request('/api/fields', {'file':'motion','values':{'x':'1'},'confirm':'save_config'})[0],409)
        self.assertEqual([r["cmd"] for r in self.state.requests],["calibration_status","calibration_status"])
    def test_wizard_is_independent_static_route(self):
        status,data=self.request('/calibration',token=False)
        self.assertEqual(status,200);self.assertIn('calibration.js'.encode(),data)
        self.assertEqual(self.state.requests,[])
    def test_read_requires_authentication(self):
        self.assertEqual(self.request('/api/calibration',token=False)[0],401)
    def test_capabilities_use_authenticated_robot_transport(self):
        self.assertEqual(self.request('/api/capabilities',token=False)[0],401)
        self.assertEqual(self.state.requests,[])
        self.assertEqual(self.request('/api/capabilities')[0],200)
        self.assertEqual(self.state.requests,[{'cmd':'capabilities'}])
    def test_forward_action_with_session_and_id(self):
        body={'action':'jog','session':'session-1','request_id':'id-2','dx_mm':.2}
        self.assertEqual(self.request('/api/calibration',body)[0],200)
        self.assertEqual(self.state.requests[-1],dict(body,cmd='placement_calibration'))
        self.assertFalse(self.state.running)
    def test_unknown_action_never_reaches_robot(self):
        self.assertEqual(self.request('/api/calibration',{'action':'shell'})[0],400)
        self.assertEqual(self.state.requests,[])
    def test_busy_action_is_not_queued(self):
        self.state.running=True
        self.assertEqual(self.request('/api/calibration',{'action':'prepare'})[0],409)
        self.assertEqual(self.state.requests,[])
    def test_config_cannot_change_during_teaching(self):
        self.state.active=True
        self.assertEqual(self.request('/api/config',{'file':'motion','content':'bad'})[0],409)
        self.assertEqual([p['cmd'] for p in self.state.requests],['calibration_status'])
    def test_frozen_image_requires_matching_revision(self):
        self.assertEqual(self.request('/api/calibration/image?session=session-1&revision=2')[0],409)
    def test_frozen_image_cannot_escape_calibration_directory(self):
        self.assertNotEqual(self.request('/api/calibration/image?session=session-1&revision=3')[0],200)
    def test_images_follow_robot_runtime_across_restarts(self):
        with tempfile.TemporaryDirectory() as folder:
            for session in ['runtime-one','runtime-two']:
                self.state.runtime=Path(folder)/session
                directory=self.state.runtime/'debug_picture/calibration/session-1'
                directory.mkdir(parents=True)
                (directory/'frame_3.jpg').write_bytes(('hand-'+session).encode())
                (directory/'head.jpg').write_bytes(('head-'+session).encode())
                status,data=self.request('/api/calibration/image?session=session-1&revision=3')
                self.assertEqual((status,data),(200,('hand-'+session).encode()))
                status,data=self.request('/api/calibration/image?kind=head&session=session-1&revision=2')
                self.assertEqual((status,data),(200,('head-'+session).encode()))
    def test_actions_are_filtered_by_current_cpp_catalog(self):
        status,data=self.request('/api/actions')
        self.assertEqual(status,200)
        actions=[a for group in json.loads(data)['groups'] for a in group['actions']]
        self.assertEqual([a['name'] for a in actions],['home'])
        self.state.mode='production'
        self.assertFalse(json.loads(self.request('/api/actions')[1])['groups'][0]['actions'][0]['allowed'])
    def test_profile_changes_require_maintenance_and_no_calibration(self):
        from unittest.mock import patch
        with patch.object(web.PROFILE_STORE,'update') as update:
            self.state.mode='production'
            self.assertEqual(self.request('/api/profiles',{'action':'select'})[0],400)
            self.state.mode='maintenance';self.state.active=True
            self.assertEqual(self.request('/api/profiles',{'action':'select'})[0],400)
            update.assert_not_called()
            self.state.active=False
            update.return_value={'ok':True,'restart_required':True}
            self.assertEqual(self.request('/api/profiles',{'action':'select'})[0],200)
            update.assert_called_once()

if __name__=='__main__':unittest.main()
