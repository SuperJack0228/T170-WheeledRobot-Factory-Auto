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
        ends = [r for r in rows if r["event"] == "end"]
        final = ends[-1] if ends else controls[-1] if controls else rows[-1]
        commits = [r for r in controls if r["phase"] == "commit"]
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
            "vision_events": dict(vision_events),
            "pair_invalidated_before_gripper": bool(vision_events.get("peer_failed_before_gripper")),
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
        if s["gripper_checks"]:
            print("  gripper:", s["gripper_checks"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
