"""Check shipped MDH/frame data against the unchanged URDF joint chains."""
from pathlib import Path
import importlib.util
import math
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('t170_model_generator', ROOT/'tools/generate_arm_model.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


class ArmModelGeometryTest(unittest.TestCase):
    def test_shipped_models_follow_full_urdf_not_just_link_lengths(self):
        rng = np.random.default_rng(124)
        for model_id in ['t170c_v2_3kg', 't170c_v2_5kg']:
            folder = ROOT/'config/robot_models'/model_id
            sdk = yaml.safe_load((folder/'Robot_Model.yaml').read_text())['T7_170']
            conventions = yaml.safe_load((folder/'joint_conventions.yaml').read_text())
            frames = yaml.safe_load((folder/'frames.yaml').read_text())
            joints = {n.get('name'): n for n in ET.parse(folder/'robot.urdf').getroot().findall('joint')}
            for side in ['left', 'right']:
                c, f, m = conventions[side], frames[side], sdk[side]
                rows = np.column_stack([m[k] for k in ['alpha', 'a', 'd', 'offset']])
                chain = [joints[n] for n in c['joint_names']]
                for u in rng.uniform(c['urdf_joint_min_rad'], c['urdf_joint_max_rad'], size=(50, 7)):
                    q = (u-c['urdf_from_sdk_offset_rad'])/c['urdf_from_sdk_direction']
                    t = np.array(f['waist_y_from_sdk_base']) @ geometry.fk_mdh(rows, q) @ np.array(f['sdk_flange_from_urdf_wrist'])
                    np.testing.assert_allclose(t, geometry.fk_urdf(chain, u), rtol=0, atol=1e-11)

    def test_three_kg_axis_tilt_and_wrist_offset_are_preserved(self):
        folder = ROOT/'config/robot_models/t170c_v2_3kg'
        sdk = yaml.safe_load((folder/'Robot_Model.yaml').read_text())['T7_170']
        self.assertGreater(abs(sdk['left']['alpha'][6]+math.pi/2), .004)
        self.assertGreater(abs(sdk['right']['offset'][4]), .004)
        self.assertAlmostEqual(abs(sdk['left']['d'][5]), .0001, places=8)
        self.assertAlmostEqual(abs(sdk['right']['d'][5]), .0001, places=8)
        # J7 nominal range differs between payload variants too.
        c3 = yaml.safe_load((folder/'joint_conventions.yaml').read_text())
        c5 = yaml.safe_load((ROOT/'config/robot_models/t170c_v2_5kg/joint_conventions.yaml').read_text())
        self.assertEqual(c3['right']['urdf_joint_max_rad'][6], 2.23)
        self.assertEqual(c5['right']['urdf_joint_max_rad'][6], 1.75)


if __name__ == '__main__':
    unittest.main()
