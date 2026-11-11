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
    def robot_request(self, payload, timeout=None):
        self.requests.append(dict(payload))
        if payload['cmd'] == 'calibration_status':
            return {'ok':True,'active':self.active,'session':'session-1','image_revision':3,
                    'image_path':'../../etc/passwd'}
        return {'ok':True,'session':'session-1','phase':'prepared'}
    def begin_action(self, title):
        if self.running:return False
        self.running=True
        return True
    def end_action(self):self.running=False
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
    def test_wizard_is_independent_static_route(self):
        status,data=self.request('/calibration',token=False)
        self.assertEqual(status,200);self.assertIn('calibration.js'.encode(),data)
        self.assertEqual(self.state.requests,[])
    def test_read_requires_authentication(self):
        self.assertEqual(self.request('/api/calibration',token=False)[0],401)
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

if __name__=='__main__':unittest.main()
