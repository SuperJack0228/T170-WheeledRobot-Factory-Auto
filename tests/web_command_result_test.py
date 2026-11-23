"""Real Web HTTP/RPC and C++ result serialization; no robot devices."""
import argparse
import importlib.util
import json
from pathlib import Path
import socketserver
import subprocess
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
PROBE,RUNTIME=sys.argv[1:3]
sys.argv=sys.argv[:1]
spec=importlib.util.spec_from_file_location('command_web',ROOT/'web_debug/server.py')
web=importlib.util.module_from_spec(spec);spec.loader.exec_module(web)

class CommandResultTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cpp_failure=json.loads(subprocess.check_output([PROBE,RUNTIME,'--emit-failure'],text=True))
        cls.replies={};cls.requests=[]
        class RPC(socketserver.StreamRequestHandler):
            def handle(self):
                request=json.loads(self.rfile.readline());cls.requests.append(request)
                reply=cls.replies.get(request['cmd'],{'ok':True})
                if reply is not None:self.wfile.write(json.dumps(reply).encode()+b'\n')
        cls.rpc=socketserver.ThreadingTCPServer(('127.0.0.1',0),RPC)
        cls.rpc.daemon_threads=True
        cls.rpc_thread=threading.Thread(target=cls.rpc.serve_forever,daemon=True);cls.rpc_thread.start()
        cls.temp=tempfile.TemporaryDirectory();web.RUNTIME_ROOT=Path(cls.temp.name)
        cls.state=web.AppState(argparse.Namespace(robot_host='127.0.0.1',robot_port=cls.rpc.server_address[1],robot_command='',token='test'))
        cls.http=web.DebugHTTPServer(('127.0.0.1',0),cls.state)
        cls.http_thread=threading.Thread(target=cls.http.serve_forever,daemon=True);cls.http_thread.start()
    @classmethod
    def tearDownClass(cls):
        cls.http.shutdown();cls.http.server_close();cls.http_thread.join()
        cls.rpc.shutdown();cls.rpc.server_close();cls.rpc_thread.join();cls.temp.cleanup()
    def setUp(self):
        self.replies.clear();self.requests.clear()
        with self.state.job_lock:self.state.job={'running':False,'title':'','step':'','error':''}
    def command(self,cmd='vision_grasp_once'):
        request=urllib.request.Request(f'http://127.0.0.1:{self.http.server_port}/api/command',
            data=json.dumps({'cmd':cmd,'confirm':cmd}).encode(),headers={'X-T170-Token':'test','Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(request,timeout=3) as reply:return reply.status,json.load(reply)
        except urllib.error.HTTPError as reply:return reply.code,json.load(reply)
    def test_real_cpp_failure_remains_failure_through_http_and_job(self):
        self.replies['vision_grasp_once']=self.cpp_failure
        status,result=self.command()
        self.assertEqual(status,200);self.assertIs(result['ok'],False)
        self.assertEqual(result['native_code'],-30);self.assertEqual(result['failure_stage'],'align_descend')
        self.assertEqual(result['skill_result']['error_code'],1302)
        self.assertEqual(self.state.job_view()['outcome'],'failed')
        self.assertIn('-30',self.state.job_view()['error'])
        self.assertEqual(len(self.requests),1)
    def test_legacy_zero_grasps_is_not_success(self):
        self.replies['vision_grasp_once']={'ok':True,'code':2,'grasped_count':0}
        _,result=self.command()
        self.assertIs(result['ok'],False);self.assertEqual(result['status'],'failed')
        self.assertTrue(self.state.job_view()['error'])
    def test_partial_and_no_target_are_distinct(self):
        for code,count,status in [(2,1,'partial'),(1,0,'no_target')]:
            self.replies['vision_grasp_once']={'ok':True,'code':code,'grasped_count':count}
            _,result=self.command()
            self.assertIs(result['ok'],False);self.assertEqual(result['status'],status)
    def test_confirmed_success_is_preserved(self):
        self.replies['vision_grasp_once']={'ok':True,'code':0,'grasped_count':2}
        _,result=self.command()
        self.assertIs(result['ok'],True);self.assertEqual(self.state.job_view()['outcome'],'succeeded')
    def test_rpc_failure_stays_visible_and_releases_job(self):
        self.replies['vision_grasp_once']=None
        status,_=self.command();self.assertEqual(status,503)
        self.assertFalse(self.state.job_view()['running']);self.assertTrue(self.state.job_view()['error'])
    def test_sequence_cannot_continue_to_belt_after_zero_grasps(self):
        self.replies['vision_grasp_once']={'ok':True,'code':2,'grasped_count':0}
        self.state.begin_action('手动编排');self.state.run_sequence(['load'])
        self.assertEqual([r['cmd'] for r in self.requests],['vision_grasp_once'])
        self.assertEqual(self.state.job_view()['outcome'],'failed')

if __name__=='__main__':unittest.main()
