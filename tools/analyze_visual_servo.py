#!/usr/bin/env python3
"""Summarize dedicated wrist-servo CSVs using only the Python standard library.

Residuals are controller/model quantities, not independent physical accuracy measurements.
"""
import argparse
import collections
import csv
import json
import math
from pathlib import Path


def number(row, key, default=0.0):
    try:
        value = float(row[key])
        return value if math.isfinite(value) else default
    except (KeyError, ValueError, TypeError):
        return default


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    index = (len(values) - 1) * q
    lo = int(index)
    return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (index - lo)


def summarize(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = {"session", "hand", "event", "phase", "steady_s", "reason"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: 不是新 visual_servo CSV，请勿混用旧 hand_shadow.csv")
        groups = collections.defaultdict(list)
        for row in reader:
            if row.get("hand") not in {"0", "1"} or not (row.get("session") or "").isdigit():
                continue
            groups[(row["session"], row["hand"])].append(row)
    result = []
    for (session, hand), rows in groups.items():
        controls = [r for r in rows if r["event"] in {"control", "end"}]
        accepted = [r for r in rows if r["event"] == "observation" and r["reason"] == "accepted"]
        rejected = collections.Counter(r["reason"] for r in rows if r["event"] == "observation" and r["reason"] != "accepted")
        vision_events = collections.Counter(r["reason"] for r in rows if r["event"] == "vision")
        virtual = [r for r in accepted if r.get("virtual_target") == "1"]
        placement_geometry = None if not virtual else {
            "sources": dict(collections.Counter(r.get("vision_source","unknown") for r in virtual)),
            "target_identities": sorted({(int(number(r,"target_row")),int(number(r,"target_col"))) for r in virtual}),
            "min_neighbor_count": min(number(r,"neighbor_count") for r in virtual),
            "max_marker_corner_count": max(number(r,"marker_corner_count") for r in virtual),
            "reprojection_p95_px": percentile([number(r,"plane_reprojection_px") for r in virtual],.95),
            "geometry_processing_p95_ms": percentile([number(r,"infer_ms") for r in virtual],.95),
        }
        ends = [r for r in rows if r["event"] == "end"]
        final = ends[-1] if ends else controls[-1] if controls else rows[-1]
        commits = [r for r in controls if r["phase"] == "commit"]
        retract_starts = [r for r in rows if r["event"] == "retract_start"]
        retract_ends = [r for r in rows if r["event"] == "retract_end"]
        retract_samples = [r for r in rows if r["event"] == "retract_sample"]
        retract_aborts = [r["reason"] for r in rows if r["event"] == "retract_abort"]
        retract_start = retract_starts[0] if retract_starts else None
        retract_end = retract_ends[-1] if retract_ends else None
        gripper_ends = [r for r in rows if r["event"] == "gripper_end" and
                        (retract_start is None or number(r,"steady_s")<=number(retract_start,"steady_s"))]
        startup = [r for r in retract_samples if retract_start is not None and
                   number(r,"steady_s")-number(retract_start,"steady_s")<=.25]
        def following_gap(row):
            if not row or row.get("joint_feedback_valid") != "1":
                return None
            try:
                gaps = [abs(float(row[f"actual_q{j}"])-float(row[f"command_q{j}"])) for j in range(7)]
                return max(gaps) if all(math.isfinite(x) for x in gaps) else None
            except (KeyError,ValueError,TypeError):
                return None
        retract = None if retract_start is None else {
            "playback_completed": retract_end is not None and not retract_aborts,
            "abort_reasons": retract_aborts,
            "duration_s": number(retract_end,"steady_s")-number(retract_start,"steady_s") if retract_end else None,
            "gripper_end_to_start_ms": 1000*(number(retract_start,"steady_s")-number(gripper_ends[-1],"steady_s")) if gripper_ends else None,
            "hold_release_wait_ms": number(retract_start,"hold_release_wait_ms"),
            "first_command_gap_rad": number(retract_start,"command_step_rad"),
            "startup_peak_command_step_rad": max((number(r,"command_step_rad") for r in startup),default=None),
            "startup_peak_command_speed_rad_s": max((number(r,"command_speed_rad_s") for r in startup),default=None),
            "sampled_command_dt_p95_ms": percentile([number(r,"dt_ms") for r in retract_samples if number(r,"dt_ms")>0],.95),
            "start_joint_following_gap_rad": following_gap(retract_start),
            "end_joint_following_gap_rad": following_gap(retract_end),
        }
        start = min(number(r, "steady_s") for r in rows)
        result.append({
            "file": str(path), "session": session, "hand": "left" if hand == "0" else "right",
            "map_mode": next((r["map_mode"] for r in rows if r.get("map_mode")), "unknown"),
            "task_tag": next((r["task_tag"] for r in rows if r.get("task_tag")), "unknown"),
            "target_class": next((r["target_class"] for r in rows if r.get("target_class") not in {None, "", "-1"}), "unknown"),
            "servo_completed": bool(ends and final["phase"] == "done"),
            "end_phase": final["phase"], "end_reason": final["reason"],
            "duration_s": number(final, "steady_s") - start,
            "first_commit_elapsed_s": number(commits[0], "steady_s") - start if commits else None,
            "accepted_frames": len(accepted), "observation_rejections": dict(rejected),
            "relaxed_grasp": final["reason"] == "ready_grasp_tolerance",
            "min_reference_height_mm": min((1000*(number(r,"reference_z_m")-number(r,"actual_z_m")+
                number(r,"height_m")) for r in controls), default=None),
            "min_actual_height_mm": min((1000*number(r,"height_m") for r in controls), default=None),
            "done_command_speed_mm_s": 1000*math.sqrt(sum(number(final,key)**2 for key in
                ("command_vx_m_s","command_vy_m_s","command_vz_m_s"))) if ends else None,
            "timeline": [{"event":r["event"], "reason":r["reason"],
                "elapsed_s":number(r,"steady_s")-start} for r in rows if r["event"] in
                {"handoff","joint_policy","both_ready","gripper_gate","gripper_start","gripper_end",
                 "retract_start","retract_end","retract_abort"}],
            "retract": retract,
            "placement_geometry": placement_geometry,
            "release_confirmed": any(r["event"]=="gripper_end" and r["reason"]=="open_confirmed" for r in rows),
            "release_failed": any(r["event"]=="gripper_end" and r["reason"]=="open_failed" for r in rows),
            "vision_events": dict(vision_events),
            "pair_invalidated_before_gripper": bool(vision_events.get("peer_failed_before_gripper")),
            "gripper_gate_rejections": [r["reason"] for r in rows
                if r["event"] == "gripper_gate" and r["reason"] != "ready"],
            "accepted_image_to_use_p95_ms": percentile([
                1000 * (number(r, "steady_s") - number(r, "capture_s")) for r in accepted], .95),
            "infer_p95_ms": percentile([number(r, "infer_ms") for r in accepted], .95),
            "control_dt_p95_ms": percentile([number(r, "dt_ms") for r in controls], .95),
            "control_dt_max_ms": max((number(r, "dt_ms") for r in controls), default=None),
            "read_p95_ms": percentile([number(r, "read_ms") for r in controls], .95),
            "solve_p95_ms": percentile([number(r, "solve_ms") for r in controls], .95),
            "joint_scaling_samples": sum(0 < number(r, "applied_scale") < 1 for r in controls),
            "solver_failures_max_consecutive": max((number(r, "solver_failures") for r in controls), default=0),
            "estimated_final_xy_error_mm": (math.hypot(number(final, "remaining_x_m"), number(final, "remaining_y_m")) * 1000) if accepted else None,
            "final_actual_z_m": number(final, "actual_z_m"),
            "gripper_checks": [
                {"outcome": r["reason"], "q_rad": number(r, "gripper_position_rad"),
                 "torque_nm": number(r, "gripper_torque_nm")}
                for r in rows if r["event"] == "gripper_check"
            ],
            "dropped_rows": max((number(r, "dropped_rows") for r in rows), default=0),
        })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path, help="CSV文件或logs/visual_servo目录")
    parser.add_argument("--json", type=Path, help="写出汇总JSON")
    args = parser.parse_args()
    files = []
    for path in args.paths:
        files.extend(sorted(path.glob("servo_*.csv")) if path.is_dir() else [path])
    if not files:
        parser.error("没有找到 visual_servo CSV")
    summaries = [item for path in files for item in summarize(path)]
    report = {"note": "servo_completed仅表示模型内闭环完成；holding_candidate也不等于已独立验证抓牢。", "sessions": summaries}
    if args.json:
        args.json.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for s in summaries:
        error = f"{s['estimated_final_xy_error_mm']:.2f}" if s["estimated_final_xy_error_mm"] is not None else "n/a"
        print(f"file={Path(s['file']).name}")
        print(f"session={s['session']} hand={s['hand']} completed={s['servo_completed']} "
              f"phase={s['end_phase']} reason={s['end_reason']} "
              f"accepted={s['accepted_frames']} model_error_mm={error}")
        print(f"  image_to_use_p95_ms={s['accepted_image_to_use_p95_ms']} "
              f"dt_p95_ms={s['control_dt_p95_ms']} solve_p95_ms={s['solve_p95_ms']}")
        if s["observation_rejections"] or s["vision_events"]:
            print("  observations:", s["observation_rejections"], "vision:", s["vision_events"])
        if s["timeline"]:
            print("  timeline:", s["timeline"])
        if s["gripper_checks"]:
            print("  gripper:", s["gripper_checks"])
        if s["retract"] is not None:
            print("  retract (command diagnostics; not measured vibration):",s["retract"])
        if s["placement_geometry"] is not None:
            print("  placement geometry:",s["placement_geometry"],
                  "open_confirmed=",s["release_confirmed"],"open_failed=",s["release_failed"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
