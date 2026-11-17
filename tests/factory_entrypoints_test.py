"""Exercise launch scripts against an inert executable and compare the real catalog."""
import importlib.util
import ast
import json
import os
import re
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
POLICY = Path(sys.argv.pop(1))

class FactoryEntrypoints(unittest.TestCase):
    def test_legacy_command_coverage_and_client_aliases(self):
        catalog = json.loads(subprocess.check_output([str(POLICY), "--dump"], text=True))
        names = {entry["name"] for entry in catalog}
        old = set(json.loads((ROOT / "tests/fixtures/factory_commands_before_split.json").read_text()))
        self.assertEqual(names, old | {"capabilities"})
        spec = importlib.util.spec_from_file_location("debug_client", ROOT / "tools/debug_client.py")
        client = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(client)
        active = {entry["name"] for entry in catalog if entry["group"] != "retired"}
        self.assertTrue({request["cmd"] for request in client.COMMANDS.values()} <= active)
        tree=ast.parse((ROOT / "web_debug/server.py").read_text())
        allowed=next(ast.literal_eval(node.value) for node in tree.body if isinstance(node,ast.Assign)
                     and any(isinstance(target,ast.Name) and target.id=="ALLOWED_COMMANDS" for target in node.targets))
        self.assertTrue(allowed <= names, f"web exposes commands without implementation: {allowed-names}")
        buttons=set(re.findall('data-command="([^"]+)"',(ROOT / "web_debug/static/index.html").read_text()))
        self.assertTrue(buttons <= names, f"buttons without implementation: {buttons-names}")

    def test_explicit_launchers_forward_mode_root_and_arguments(self):
        with tempfile.TemporaryDirectory(prefix="factory entry ") as temp:
            root = Path(temp)
            (root / "build_robot").mkdir()
            executable = root / "build_robot/t170_factory"
            executable.write_text('#!/bin/bash\nprintf \'%s\\n\' "$PWD" "$T170_ROOT" "$@"\n')
            executable.chmod(0o755)
            for script, mode in (("run_production.sh", "--production"), ("run_maintenance.sh", "--maintenance")):
                result = subprocess.check_output(["bash", str(ROOT / "deploy" / script), "--port", "8123"],
                    text=True, env=dict(os.environ, T170_ROOT=str(root)))
                self.assertEqual(result.splitlines(), [str(root), str(root), mode, "--port", "8123"])

    def test_default_service_selects_production(self):
        service = (ROOT / "deploy/t170-factory.service").read_text()
        self.assertIn("/deploy/run_production.sh", service)
        self.assertNotIn("--maintenance", service)

if __name__ == "__main__":
    unittest.main()
