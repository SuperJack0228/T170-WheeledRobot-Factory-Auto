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
            # Resolve the relocated SDK too, so forbidden public SDK imports
            # remain visible to boundary checks after dependency consolidation.
            for directory in (ROOT / "include", ROOT / "dependencies/sdk/arm/include"):
                candidate = directory / name
                if candidate.is_file():
                    self.includes(candidate, seen)
                    break
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

    def test_motion_public_headers_do_not_expose_sdk_or_app(self):
        for header in (ROOT / "include/t170/motion").glob("*.h"):
            self.assertNotIn("using namespace", header.read_text())
            for path in self.includes(header):
                self.assertNotIn(path.name, ("function.h", "head.h", "Ti5_Arm.h", "Ti5_Robotics.h"))
                self.assertNotIn("/t170/app/", str(path))

    def test_motion_implementations_do_not_depend_on_legacy_facade(self):
        for source in (ROOT / "src/motion").rglob("*"):
            if source.suffix in (".cpp", ".h"):
                self.assertNotIn('#include "function.h"', source.read_text(), str(source))
                self.assertNotIn('t170/app/diagnostic_gestures.h', source.read_text())

    def test_motion_build_has_one_implementation_per_compatibility_path(self):
        cmake = (ROOT / "CMakeLists.txt").read_text()
        manifest = (ROOT / "cmake/T170Motion.cmake").read_text()
        self.assertNotIn("src/function.cpp", cmake)
        self.assertNotIn("src/servo/runtime.cpp", cmake)
        self.assertIn("$<TARGET_OBJECTS:t170_motion>", cmake)
        self.assertIn("${T170_SERVO_RUNTIME_SOURCES}",
                      (ROOT / "cmake/T170OfflineTests.cmake").read_text())
        sources = re.findall(r"src/motion/[A-Za-z0-9_]+\.cpp", manifest)
        self.assertEqual(len(sources), len(set(sources)))
        for name in ("kinematics_adapter", "planning", "trajectory_executor", "feedback",
                     "retreat", "servo_handoff", "servo_executor", "final_hold"):
            self.assertIn(f"src/motion/{name}.cpp", sources)

    def test_motion_shared_state_has_one_definition(self):
        sources = list((ROOT / "src/motion").glob("*.cpp"))
        text = "\n".join(p.read_text() for p in sources)
        for pattern in (r"^std::mutex g_arm_kin_mu\[2\];", r"^Runtime slots\[2\];",
                        r"^int g_arm_last_cmd_cnt\[2\]\[7\] = \{\};"):
            self.assertEqual(len(re.findall(pattern, text, re.MULTILINE)), 1, pattern)

    def test_standard_skill_headers_do_not_expose_sdk_or_legacy(self):
        for name in ("types.h", "tray.h", "conveyor.h"):
            header = ROOT / "include/t170/skills" / name
            self.assertNotIn("using namespace", header.read_text())
            for path in self.includes(header):
                self.assertNotIn(path.name, ("function.h", "head.h", "Ti5_Arm.h", "move_box_runtime.h"))
                self.assertNotIn("/skills/legacy/", str(path))

    def test_skill_implementations_do_not_import_dispatch_or_chassis_protocol(self):
        for source in (ROOT / "src/skills").rglob("*.cpp"):
            for forbidden in ('"robot_runtime.h"', '"robot_client.hpp"', '"Ti5_Seer.hpp"'):
                self.assertNotIn(forbidden, source.read_text(), str(source))

    def test_legacy_skill_sources_have_no_second_compiled_implementation(self):
        cmake = (ROOT / "CMakeLists.txt").read_text()
        for source in ("src/move_box_runtime.cpp", "src/grasp_to_standby.cpp", "src/grasp_execution.cpp"):
            self.assertNotIn(source, cmake)
        self.assertIn("$<TARGET_OBJECTS:t170_skills>", cmake)
        manifest = (ROOT / "cmake/T170Skills.cmake").read_text()
        sources = re.findall(r"src/skills/[A-Za-z0-9_/]+\.cpp", manifest)
        self.assertEqual(len(sources), len(set(sources)))

    def test_cpp_bridge_uses_namespaced_python_modules(self):
        source = (ROOT / "src/vision/bridge/python_bridge.cpp").read_text()
        for old in ("algorithm.engine", "tray_detect_api", "aruco_detect_api", "online"):
            self.assertNotIn(f'py::module_::import("{old}")', source)
        for name in ("detection.engine", "geometry.tray.api", "geometry.conveyor.api", "geometry.marker.api"):
            self.assertIn(f'py::module_::import("t170_vision.{name}")', source)

    def test_vendor_protocol_headers_are_confined_to_adapters(self):
        allowed = {"robot_client.hpp": "src/integration/dispatch/robot_client_adapter.cpp",
                   "Ti5_Seer.hpp": "src/hardware/chassis_seer.cpp"}
        for source in (ROOT / "src").rglob("*.cpp"):
            for header, owner in allowed.items():
                if f'#include "{header}"' in source.read_text():
                    self.assertEqual(source.relative_to(ROOT).as_posix(),owner)
        for source in (ROOT / "src/tasks").rglob("*"):
            if source.suffix in (".cpp", ".h"):
                self.assertNotIn("RobotClient::",source.read_text())
                self.assertNotIn("ti5::seer::",source.read_text())

    def test_factory_public_ports_do_not_import_vendor_or_app(self):
        for name in ("tasks/context.h", "tasks/factory.h", "hardware/chassis.h",
                     "integration/dispatch/client.h", "integration/dispatch/inbox.h"):
            for header in self.includes(ROOT / "include/t170" / name):
                self.assertNotIn(header.name,("robot_client.hpp","Ti5_Seer.hpp","Ti5_Arm.h","function.h"))
                self.assertNotIn("/t170/app/",str(header))

    def test_assembly_and_command_gates_precede_hardware_actions(self):
        main=(ROOT/"apps/t170_factory_main.cpp").read_text()
        self.assertIn("t170::app::run(argc,argv)",main)
        self.assertLess(len(main.splitlines()),10)
        router=(ROOT/"src/app/command_router.cpp").read_text()
        self.assertLess(router.index("command_allowed"),router.index("g_hw_mu"))
        self.assertLess(router.index("command_allowed"),router.index("hardware_abort_clear"))
        assembly=(ROOT/"src/app/application.cpp").read_text()
        self.assertLess(assembly.index("options.help"),assembly.index("init_hardware"))
        self.assertLess(assembly.index("options.list_commands"),assembly.index("init_hardware"))

    def test_no_compiled_legacy_factory_implementations(self):
        cmake=(ROOT/"CMakeLists.txt").read_text()
        for old in ("orchestrator/cpp/robot_runtime.cpp","orchestrator/cpp/dispatch_board.cpp",
                    "src/common/task_timing.cpp","src/move_box_config.cpp"):
            self.assertNotIn(old,cmake)
        manifest=(ROOT/"cmake/T170Factory.cmake").read_text()
        for folder,variable in (("tasks","T170_TASK_SOURCES"),("app","T170_APPLICATION_SOURCES")):
            listed=re.findall(r"src/"+folder+r"/[A-Za-z0-9_/]+\.cpp",manifest)
            self.assertEqual(len(listed),len(set(listed)))
            self.assertIn(variable,cmake)
        self.assertNotIn("tests/vendor_doubles",cmake)

if __name__ == "__main__":
    unittest.main()
