"""Real package/config/geometry imports; no camera startup, weights or inference."""
import ast
import importlib
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

class VisionPackage(unittest.TestCase):
    def test_canonical_imports_from_other_working_directory(self):
        env = os.environ.copy()
        env["PYTHONPATH"] = str(ROOT / "python") + os.pathsep + env.get("PYTHONPATH", "")
        code = """
import sys
import t170_vision
from t170_vision.geometry.conveyor import api as belt
from t170_vision.geometry.tray import api as tray
from t170_vision.geometry.marker import api as marker
from t170_vision.detection import config_loader, pose_box, pose_pnp, pose_centroid
assert 'online' not in sys.modules and 'algorithm' not in sys.modules
assert 'torch' not in sys.modules and 'ultralytics' not in sys.modules
tray.init(); belt.init(); marker.init()
assert tray._cfg.dictionary.startswith('DICT_5X5')
assert belt._cfg.dictionary.startswith('DICT_5X5')
assert tray._cfg is not belt._cfg
"""
        with tempfile.TemporaryDirectory() as cwd:
            proc = subprocess.run([sys.executable, "-c", code], cwd=cwd, env=env,
                                  capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)

    def test_legacy_entry_points_delegate_to_same_classes(self):
        from t170_vision.geometry.tray import api
        from t170_vision.detection import config_loader
        for path, name, expected in [
            ("board_yf100_aruco/tray_detect_api.py", "legacy_tray_api_probe", api),
            ("feeding_cylindrical_parts_alg/algorithm/config_loader.py", "legacy_config_probe", config_loader),
        ]:
            spec = importlib.util.spec_from_file_location(name, ROOT / path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
            self.assertIs(sys.modules[name], expected)
        self.assertEqual(api.HERE, ROOT / "board_yf100_aruco")
        self.assertEqual(config_loader.DEFAULT_CONFIG_PATH,
                         ROOT / "feeding_cylindrical_parts_alg/config/pose_params.yaml")

    def test_missing_real_model_is_explicit(self):
        import yaml
        from t170_vision.detection.config_loader import load_pose_params
        cfg = yaml.safe_load((ROOT / "feeding_cylindrical_parts_alg/config/pose_params.yaml").read_text())
        self.assertEqual(cfg["segmentation"]["model"], "models/best1015.pt")
        self.assertEqual(cfg["segmentation"]["hand_model"], "models/best1103.pt")
        with tempfile.TemporaryDirectory() as directory:
            cfg["segmentation"]["hand_model"] = str(Path(directory) / "missing-hand-model.pt")
            config = Path(directory) / "pose.yaml"
            config.write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(FileNotFoundError, "missing-hand-model"):
                load_pose_params(config)

    def test_tray_fusion_and_head_transform_math(self):
        import numpy as np
        from t170_vision.geometry.tray.fusion import _self_test
        from t170_vision.calibration.head_transform import euler_to_rotation_matrix, make_transform
        _self_test()
        rotation = euler_to_rotation_matrix(0, 0, 0)
        np.testing.assert_allclose(rotation, np.eye(3))
        matrix = make_transform(rotation, [10, 20, 30])
        np.testing.assert_allclose(matrix[:3, 3], [10, 20, 30])

    def test_all_migrated_python_is_syntactically_valid(self):
        for path in (ROOT / "python/t170_vision").rglob("*.py"):
            ast.parse(path.read_text(), filename=str(path))

if __name__ == "__main__":
    unittest.main()
