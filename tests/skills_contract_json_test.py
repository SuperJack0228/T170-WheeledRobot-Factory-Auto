"""Parse the real C++ skill output; unknown and partial effects must remain explicit."""
import json
from pathlib import Path
import subprocess
import sys

result = json.loads(subprocess.check_output([sys.argv[1], "--json"], text=True))
assert result["interface_version"] == 1
assert result["request_id"] == 'request-"1"\n'
assert result["workcell_id"] == "机床组5"
assert result["skill"] == "tray.place"
assert result["status"] == "failed"
assert result["error_code"] == 1402 and result["native_code"] == -38
assert result["failure_stage"] == "gripper"
left, right = result["hands"]["left"], result["hands"]["right"]
assert left["part_state"] == "empty" and left["action_completed"]
assert not left["retreat_completed"]
assert left["native_code"] is None
assert right["part_state"] == "unknown" and not right["action_completed"]
assert right["native_code"] == -38 and right["failure_stage"] == "gripper"
schema = json.loads((Path(__file__).resolve().parents[1] / "config/schemas/skill_result.v1.schema.json").read_text())
assert set(result) == set(schema["properties"])
for hand in (left, right):
    assert set(hand) == set(schema["$defs"]["hand"]["properties"])
    for key in ("part_state", "evidence", "stage", "error_code", "failure_stage"):
        assert hand[key] in schema["$defs"]["hand"]["properties"][key]["enum"]
assert left["nominal_target"]["frame"] == "upright_arm_base"
assert left["nominal_target"]["tray_row_aruco"] == 5
assert len(left["nominal_target"]["pose_m_rad"]) == 6
assert right["nominal_target"] is None
print("C++ skill JSON: schema identity, escaping, confirmed partial release and unknown peer passed")
