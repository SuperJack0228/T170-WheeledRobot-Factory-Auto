#!/usr/bin/env python3
"""Summarize v2 lift/return feedback without mistaking commands for arrival."""
import argparse
import csv
import json
import math
from pathlib import Path


def analyze(path):
    groups = {}
    with path.open() as stream:
        for row in csv.DictReader(stream):
            if row.get("reason") != "joint_motion_v2":
                continue
            stage = row["event"].split("_", 1)[0]
            key = (row["session"], row["hand"], stage)
            groups.setdefault(key, []).append(row)
    summaries = []
    for (session, hand, stage), rows in groups.items():
        measured = [r for r in rows if r.get("joint_feedback_valid") == "1"]
        def gap(row):
            return max(abs(float(row[f"actual_q{j}"]) - float(row[f"command_q{j}"])) for j in range(7))
        command_end = next((r for r in rows if r["event"].endswith("_command_complete")), None)
        arrived = next((r for r in rows if r["event"].endswith("_measured_arrival")), None)
        summaries.append({
            "file": str(path), "session": session, "hand": hand, "stage": stage,
            "command_complete": command_end is not None, "measured_arrival": arrived is not None,
            "last_event": rows[-1]["event"],
            "duration_s": float(rows[-1]["steady_s"]) - float(rows[0]["steady_s"]),
            "settle_after_command_s": float(arrived["steady_s"]) - float(command_end["steady_s"])
                if arrived and command_end else None,
            "peak_sampled_joint_lag_deg": math.degrees(max(map(gap, measured))) if measured else None,
            "last_joint_lag_deg": math.degrees(gap(measured[-1])) if measured else None,
            "last_actual_xyz_m": [float(measured[-1][key]) for key in
                ("actual_x_m", "actual_y_m", "actual_z_m")] if measured else None,
        })
    return summaries


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, nargs="+")
    args = parser.parse_args()
    print(json.dumps([item for path in args.csv for item in analyze(path)], indent=2, ensure_ascii=False))
