"""Exercise deployment overrides without importing vision, YAML or hardware SDKs."""
from pathlib import Path
import json
import os
import runpy
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from t170_common.paths import load_paths


class RuntimePathsTest(unittest.TestCase):
    def test_default_paths(self):
        with patch.dict(os.environ, {}, clear=True):
            root = Path(__file__).resolve().parents[1]
            paths = load_paths()
            self.assertEqual(paths.root, root)
            self.assertEqual(paths.config_dir, root / "config")
            self.assertEqual(paths.log_dir, root / "logs")

    def test_overrides_match_cpp_rules(self):
        with tempfile.TemporaryDirectory(prefix="t170 paths ") as directory:
            root = Path(directory).resolve()
            with patch.dict(os.environ, {"MARKET_SIMPLE_ROOT": str(root / "old"),
                    "T170_ROOT": str(root), "T170_CONFIG_DIR": "machine config",
                    "T170_LOG_DIR": str(root / "external logs")}, clear=True):
                paths = load_paths()
                self.assertEqual(paths.root, root)
                self.assertEqual(paths.config_dir, root / "machine config")
                self.assertEqual(paths.log_dir, root / "external logs")
                self.assertEqual(load_paths(root / "explicit").root, root / "explicit")
                os.environ["T170_ROOT"] = ""
                self.assertEqual(load_paths().root, root / "old")

    def test_web_configuration_matches_runtime_overrides(self):
        repository = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(prefix="t170 web paths ") as directory:
            root = Path(directory).resolve()
            environment = {"T170_ROOT": str(root), "T170_CONFIG_DIR": "machine",
                           "T170_LOG_DIR": "run logs"}
            # Only inspect module wiring: no YAML parsing, server, SDK or process launch.
            with patch.dict(os.environ, environment, clear=True), \
                 patch.dict(sys.modules, {"yaml": types.ModuleType("yaml")}):
                module = runpy.run_path(str(repository / "web_debug/server.py"), run_name="web_paths_test")
                self.assertEqual(module["CONFIG_FILES"]["motion"], root / "machine/move_box_params.yaml")
                self.assertEqual(module["CONFIG_FILES"]["hand_eye_fallback"], root / "machine/camera_to_base_result.yaml")
                self.assertEqual(module["RUNTIME_ROOT"], root / "run logs/web")
                with patch.object(sys, "argv", ["server.py"]):
                    self.assertEqual(module["parse_args"]().robot_command, "./build_robot/t170_factory")

    def test_launcher_preserves_arguments_root_and_working_directory(self):
        repository = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(prefix="t170 launcher ") as directory:
            root = Path(directory).resolve()
            binary = root / "build_robot/t170_factory"
            binary.parent.mkdir()
            binary.write_text("#!" + sys.executable + "\nimport json,os,sys\n"
                              "print(json.dumps([os.getcwd(),os.environ['T170_ROOT'],sys.argv[1:]]))\n")
            binary.chmod(0o755)
            with patch.dict(os.environ, {"T170_ROOT": str(root)}):
                result = subprocess.run(["bash", str(repository / "deploy/run_factory.sh"),
                                         "--port", "8099", "argument with spaces"],
                                        cwd="/", text=True, capture_output=True, check=True)
            self.assertEqual(json.loads(result.stdout),
                             [str(root), str(root), ["--port", "8099", "argument with spaces"]])


if __name__ == "__main__":
    unittest.main()
