"""Check the first-step module boundaries without loading hardware."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]

class ModuleBoundaries(unittest.TestCase):
    def includes(self, path, seen=None):
        seen = set() if seen is None else seen
        path = path.resolve()
        if path in seen:
            return seen
        seen.add(path)
        for name in re.findall(r'#include "([^"]+)"', path.read_text()):
            candidate = ROOT / "include" / name
            if candidate.is_file():
                self.includes(candidate, seen)
        return seen

    def test_control_and_trace_have_no_hardware_or_vision_dependency(self):
        for name in ("control/visual_servo.h", "common/servo_trace.h"):
            files = self.includes(ROOT / "include/t170" / name)
            for path in files:
                rel = path.relative_to(ROOT / "include").as_posix()
                self.assertFalse(rel.startswith(("t170/hardware/", "t170/vision/")), rel)
                self.assertNotIn(path.name, ("function.h", "Ti5_Arm.h", "realsense_get.h"))

    def test_observer_cannot_send_motion_commands(self):
        source = (ROOT / "src/vision/tracking/wrist_observer.cpp").read_text()
        for call in ("set_motor_position", "send_joint_reference", "socketcan_sendcommand", "run_descent"):
            self.assertNotRegex(source, rf"\b{call}\s*\(")
        self.assertNotIn('#include "servo/runtime.h"', source)
        self.assertIn('"t170/vision/servo_observer_port.h"', source)

    def test_build_paths_exist(self):
        for path in [ROOT / "CMakeLists.txt", *ROOT.glob("cmake/T170*.cmake")]:
            for source in re.findall(r"\b(?:src|tests|apps)/[A-Za-z0-9_/.-]+\.cpp", path.read_text()):
                self.assertTrue((ROOT / source).is_file(), source)

    def test_cpp_bridge_uses_namespaced_python_modules(self):
        source = (ROOT / "src/vision/bridge/python_bridge.cpp").read_text()
        for old in ("algorithm.engine", "tray_detect_api", "aruco_detect_api", "online"):
            self.assertNotIn(f'py::module_::import("{old}")', source)
        for name in ("detection.engine", "geometry.tray.api", "geometry.conveyor.api", "geometry.marker.api"):
            self.assertIn(f'py::module_::import("t170_vision.{name}")', source)

if __name__ == "__main__":
    unittest.main()
