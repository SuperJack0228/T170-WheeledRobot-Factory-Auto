#!/usr/bin/env python3
"""Read-only migration audit against a local Git baseline; does not access hardware/network.

This is a one-time migration comparison, not a permanent requirement that algorithms
never change. Run it before the next intentional behavior change or select its baseline.
"""
import argparse
import ast
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]

def git(*args):
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, check=True).stdout

def cpp_tokens(text):
    text = re.sub(r"^\s*#include[^\n]*", "", text, flags=re.M)
    return re.findall(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*.*?\*/|[^\s]', text, re.S)

def function(text, signature):
    start = text.index(signature)
    i = text.index("{", start) + 1
    depth = 1
    while depth:
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
        i += 1
    return text[start:i]

class PythonAlgorithm(ast.NodeTransformer):
    def visit_Import(self, node):
        return None
    def visit_ImportFrom(self, node):
        return None
    def visit_Expr(self, node):
        if isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
            return None  # documentation is not an algorithm
        return self.generic_visit(node)
    def visit_FunctionDef(self, node):
        if node.name == "_load_online":
            return None  # replaced by a package-relative import of the same detector
        return self.generic_visit(node)
    def visit_Assign(self, node):
        names = {x.id for x in node.targets if isinstance(x, ast.Name)}
        if names & {"HERE", "PACKAGE_DIR", "PROJECT_ROOT", "_ONLINE_NAME"}:
            return None  # resource roots remain at their old locations via load_paths()
        if "online" in names and isinstance(node.value, ast.Call):
            return None
        return self.generic_visit(node)

def python_algorithm(text):
    return ast.dump(PythonAlgorithm().visit(ast.parse(text)), include_attributes=False)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", default="e205e9e49e4efe650a8dd140528ab29add9c12bb")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    read_old = lambda name: git("show", f"{args.baseline}:{name}")
    entries = list(csv.DictReader((ROOT / "docs/architecture/step1_file_moves.csv").open()))
    result = {"baseline": args.baseline, "configuration": [], "cpp": [], "python": [], "types": [], "failures": []}
    files = git("ls-tree", "-r", "--name-only", args.baseline).decode().splitlines()
    for name in files:
        if not name.endswith((".yaml", ".json")) or not name.startswith(("config/", "feeding_cylindrical_parts_alg/config/",
                "board_yf100_aruco/", "board_belt_aruco/", "online_pose/", "free_calib/config/")):
            continue
        old = read_old(name)
        current = (ROOT / name).read_bytes()
        same = old == current
        result["configuration"].append({"file": name, "byte_identical": same,
            "sha256": hashlib.sha256(current).hexdigest()})
        if not same:
            result["failures"].append(name)
    cpp = {x["source"]: x["destination"] for x in entries if x["source"].endswith(".cpp")}
    cpp.update({x: x for x in ("src/function.cpp", "src/grasp_to_standby.cpp", "src/grasp_execution.cpp",
        "src/move_box_runtime.cpp", "src/move_box_config.cpp", "src/servo/runtime.cpp", "orchestrator/cpp/robot_runtime.cpp")})
    for old_name, new_name in cpp.items():
        if old_name == "src/seg_pose_bridge.cpp":
            continue  # visualization split and import wiring are separately compiled/tested
        old = read_old(old_name).decode()
        new = (ROOT / new_name).read_text()
        scope = "all non-include tokens"
        if old_name == "src/vision/dl.cpp":
            # ModelRegistry moved to its own file and now records configured
            # engine paths, rather than duplicated hardcoded model filenames.
            # Its roles/paths are separately tested by model_registry_test.
            old = old[old.index("DlResult dl_from_pose_run"):]
            new = new[new.index("DlResult dl_from_pose_run"):]
            for text_name in ("old", "new"):
                text = old if text_name == "old" else new
                text = text.replace(function(text, "BridgeDlEngine::BridgeDlEngine("), "")
                text = text.replace(function(text, "bool BridgeDlEngine::load("), "")
                if text_name == "old":
                    old = text
                else:
                    new = text
            scope = "inference/conversion/routing; registry metadata binding separately tested"
        if old_name == "src/servo/visual_servo.cpp":
            common = (ROOT / "src/common/servo_types.cpp").read_text()
            for signature in ("double monotonic_seconds()", "const char *phase_name(Phase p)"):
                body = function(old, signature)
                if cpp_tokens(body) != cpp_tokens(function(common, signature)):
                    result["failures"].append(signature)
                old = old.replace(body, "")
        same = cpp_tokens(old) == cpp_tokens(new)
        result["cpp"].append({"source": old_name, "current": new_name, "scope": scope,
                              "same_noninclude_tokens": same})
        if not same:
            result["failures"].append(new_name)
    for item in entries:
        if not item["source"].endswith(".py"):
            continue
        old = read_old(item["source"]).decode()
        new = (ROOT / item["destination"]).read_text()
        same = python_algorithm(old) == python_algorithm(new)
        result["python"].append({**item, "same_algorithm_AST": same})
        if not same:
            result["failures"].append(item["destination"])
    for old_header, new_header, signatures in [
        ("include/realsense_get.h", "include/t170/common/frame.h", ["enum class CameraSlot", "struct CameraFrameData"]),
        ("include/servo/visual_servo.h", "include/t170/common/servo_types.h",
         ["struct State {", "struct PixelObservation", "struct Mapping", "enum class Phase", "struct Output"]),
        ("include/servo/visual_servo.h", "include/t170/control/visual_servo.h",
         ["struct MapKnot", "struct ImageProfile", "struct Config", "class StateHistory", "class Controller"]),
        ("include/servo/tray_localizer.h", "include/t170/common/tray_types.h",
         ["struct TrayPlaceVisionConfig", "struct TrayBoardGeometry", "struct TrayAnchor", "struct TrayTarget"]),
        ("include/seg_pose_bridge.h", "include/t170/vision/perception_types.h",
         ["struct PoseTargetResult", "struct PoseRunResult", "struct PoseDetectionRecord", "enum class SegEngineId",
          "struct ArucoMarkerResult", "struct ArucoDetectResult", "struct TrayHoleResult", "struct TrayDetectResult",
          "struct BeltDetectResult", "struct PerceiveTrayOptions", "struct HeadTrayPerception", "struct HeadBeltPerception",
          "struct HeadHandAssignZones", "struct PoseVisPanel", "struct PoseVisLayout"]),
    ]:
        old, new = read_old(old_header).decode(), (ROOT / new_header).read_text()
        for signature in signatures:
            same = cpp_tokens(function(old, signature)) == cpp_tokens(function(new, signature))
            result["types"].append({"type": signature, "same_definition_tokens": same})
            if not same:
                result["failures"].append(signature)
    print(f"configs={len(result['configuration'])}, cpp={len(result['cpp'])}, "
          f"python={len(result['python'])}, types={len(result['types'])}, failures={len(result['failures'])}")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    if result["failures"]:
        print("\n".join(result["failures"]))
        return 1
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
