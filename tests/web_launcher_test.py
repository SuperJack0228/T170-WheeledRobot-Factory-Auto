"""Real launcher + fake local RPC, with no robot/camera/model initialization."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import shutil
import socket
import socketserver
import subprocess
import sys
import tempfile
import threading
import unittest

ROOT = Path(__file__).resolve().parents[1]


@contextmanager
def factory_reply(payload):
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            request = json.loads(self.rfile.readline())
            if request != {'cmd': 'ping'}:
                self.wfile.write(b'{"ok":false}\n')
            else:
                self.wfile.write(json.dumps(payload).encode() + b'\n')
    with socketserver.TCPServer(('127.0.0.1', 0), Handler) as server:
        thread = threading.Thread(target=lambda: server.serve_forever(poll_interval=.01))
        thread.start()
        try:
            yield server.server_address[1]
        finally:
            server.shutdown()
            thread.join()


class WebLauncherTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='t170 web launcher ')
        self.base = Path(self.temporary.name)
        self.source = self.base / 'source'
        for folder in ('deploy', 'tools', 'web_debug'):
            (self.source / folder).mkdir(parents=True)
        for name in ('run_web.sh', 'runtime_context.sh'):
            shutil.copyfile(ROOT / 'deploy' / name, self.source / 'deploy' / name)
        shutil.copyfile(ROOT / 'tools/active_runtime.py', self.source / 'tools/active_runtime.py')
        (self.source / 'web_debug/server.py').write_text(
            'import json,os,sys\n'
            'print(json.dumps({"root":os.environ["T170_ROOT"],'
            '"config":os.environ["T170_CONFIG_DIR"],'
            '"cwd":os.getcwd(),"args":sys.argv[1:]}))\n')
        self.runtime = self.base / 'live runtime'
        self.config = self.runtime / 'config'
        self.config.mkdir(parents=True)
        (self.config / 'effective_params.yaml').write_text('{}\n')
        self.env = dict(os.environ, T170_PYTHON=sys.executable)
        for name in ('T170_ROOT', 'T170_RUNTIME_DIR', 'T170_CONFIG_DIR', 'T170_DEPLOYMENT', 'MARKET_SIMPLE_ROOT'):
            self.env.pop(name, None)

    def tearDown(self):
        self.temporary.cleanup()

    def run_web(self, *args):
        return subprocess.run(['bash', str(self.source / 'deploy/run_web.sh'), *args],
                              env=self.env, text=True, capture_output=True, timeout=10)

    def test_live_runtime_and_web_arguments(self):
        with factory_reply({'ok': True, 'mode': 'production', 'runtime_root': str(self.runtime),
                            'config_dir': str(self.config)}) as port:
            args = ['--robot-port', str(port), '--port', '8123', '--host', '0.0.0.0']
            result = self.run_web(*args)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertEqual(data['root'], str(self.runtime))
        self.assertEqual(data['config'], str(self.config))
        self.assertEqual(Path(data['cwd']).resolve(), self.runtime.resolve())
        self.assertEqual(data['args'], args)

    def test_custom_live_config_replaces_stale_config_environment(self):
        custom = self.base / 'custom config'
        custom.mkdir()
        (custom / 'effective_params.yaml').write_text('{}\n')
        self.env['T170_CONFIG_DIR'] = str(self.base / 'stale')
        with factory_reply({'ok': True, 'runtime_root': str(self.runtime),
                            'config_dir': str(custom)}) as port:
            result = self.run_web('--robot-port', str(port))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['config'], str(custom))

    def test_explicit_runtime_still_works_without_factory(self):
        self.env['T170_RUNTIME_DIR'] = str(self.runtime)
        result = self.run_web('--port', '8124')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['root'], str(self.runtime))

    def test_source_root_environment_uses_live_runtime(self):
        self.env['T170_ROOT'] = str(self.source)
        (self.source / 'config').mkdir()
        (self.source / 'config/deployment.yaml').write_text('manifest: old-profile.yaml\n')
        with factory_reply({'ok': True, 'runtime_root': str(self.runtime),
                            'config_dir': str(self.config)}) as port:
            result = self.run_web('--robot-port', str(port))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['root'], str(self.runtime))

    def test_old_factory_cannot_silently_select_another_runtime(self):
        with factory_reply({'ok': True, 'mode': 'maintenance'}) as port:
            result = self.run_web('--robot-port', str(port))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, '')
        self.assertIn('未提供运行目录', result.stderr)

    def test_invalid_runtime_is_rejected_before_web_execution(self):
        for value in ('relative/path', str(self.base / 'missing'), str(self.runtime) + '\nextra'):
            with self.subTest(value=value):
                with factory_reply({'ok': True, 'runtime_root': value}) as port:
                    result = self.run_web('--robot-port', str(port))
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(result.stdout, '')

    def test_no_factory_does_not_generate_a_new_runtime(self):
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        before = set(self.base.iterdir())
        result = self.run_web('--robot-port', str(port))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, '')
        self.assertEqual(set(self.base.iterdir()), before)

    def test_explicit_missing_runtime_does_not_fall_back_to_source(self):
        self.env['T170_RUNTIME_DIR']=str(self.base/'does not exist')
        result=self.run_web()
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(result.stdout,'')


if __name__ == '__main__':
    unittest.main()
