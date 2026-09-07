#!/usr/bin/env python3
"""动作编排台 HTTP + SSE 服务。

运行:
  python3 orchestrator/server.py
  浏览器打开 http://<ip>:8088

底盘相关接口不存在。硬件默认仿真。
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from concurrent.futures import ThreadPoolExecutor, wait

from flask import Flask, Response, jsonify, request, send_from_directory

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
STATIC = ROOT / "static"
SEQ_DIR = ROOT / "sequences"
SEQ_DIR.mkdir(exist_ok=True)
BOOT_FILE = SEQ_DIR / "_boot_init.json"
DEBUG_VIS = PROJECT_ROOT / "picture_debug"

sys.path.insert(0, str(ROOT))

from app.robot import Abort, create_robot  # noqa: E402
from app.runner import SequenceRunner  # noqa: E402
from app.schema import (  # noqa: E402
    LIBRARY,
    boot_standby_from_steps,
    boot_waist_from_steps,
    canonicalize_step,
    default_boot_steps,
    merge_into_sequence,
    program_document,
    validate_program,
)
from app import approach_paths as approach_paths_mod  # noqa: E402
from app import grasp_params as grasp_params_mod  # noqa: E402

app = Flask(__name__, static_folder=str(STATIC), static_url_path="/static")

_log_q: queue.Queue = queue.Queue(maxsize=2000)
_log_hist: list[dict] = []
_hist_lock = threading.Lock()
_sse_lock = threading.Lock()
_sse_queues: list[queue.Queue] = []
_state_lock = threading.Lock()
_state: dict = {}
_seq_lock = threading.Lock()
_steps: list[dict] = []
_program_name = "untitled"
_boot_lock = threading.Lock()
_boot_steps: list[dict] = []


def _cli_mode() -> str:
    mode = os.environ.get("ORCH_MODE", "hw")
    if "--mode" in sys.argv:
        i = sys.argv.index("--mode")
        if i + 1 < len(sys.argv):
            mode = sys.argv[i + 1]
    return mode


def _broadcast(event: str, data: dict) -> None:
    payload = json.dumps({"event": event, "data": data}, ensure_ascii=False)
    with _sse_lock:
        dead = []
        for q in _sse_queues:
            try:
                q.put_nowait(payload)
            except queue.Full:
                dead.append(q)
        for q in dead:
            _sse_queues.remove(q)


def emit_log(level: str, source: str, message: str) -> None:
    item = {
        "ts": datetime.now().strftime("%H:%M:%S.%f")[:-3],
        "level": level.upper(),
        "source": source,
        "message": message,
    }
    with _hist_lock:
        _log_hist.append(item)
        if len(_log_hist) > 800:
            del _log_hist[:200]
    try:
        _log_q.put_nowait(item)
    except queue.Full:
        pass
    _broadcast("log", item)
    print(f"[{item['ts']}] {item['level']:5} {source:7} {message}", flush=True)


def on_state(snap: dict) -> None:
    global _state
    with _state_lock:
        _state = snap
    _broadcast("state", snap)


abort_event = threading.Event()
robot = create_robot(_cli_mode(), emit_log, abort_event)
runner = SequenceRunner(robot, emit_log, on_state)
on_state(runner.snapshot())

_standby_reload_pending = False
_standby_move_gen = 0
_standby_move_mu = threading.Lock()


def _flush_standby_reload() -> None:
    global _standby_reload_pending
    if not _standby_reload_pending:
        return
    robot.reload_grasp_config()
    _standby_reload_pending = False


def _move_standby_now(left: dict | None, right: dict | None, speed: float, gen: int) -> None:
    if runner.is_busy():
        return
    jobs = []
    if right:
        jobs.append(("right", right, speed))
    if left:
        jobs.append(("left", left, speed))
    if not jobs:
        return

    def one(side: str, pose: dict, spd: float) -> None:
        with _standby_move_mu:
            if gen != _standby_move_gen:
                return
        robot.move_arm(side, pose, spd)

    try:
        robot.abort_event.clear()
        if len(jobs) == 1:
            one(*jobs[0])
            return
        errors: list[BaseException] = []

        def wrap(job):
            try:
                one(*job)
            except Abort:
                raise
            except Exception as exc:  # noqa: BLE001
                errors.append(exc)

        with ThreadPoolExecutor(max_workers=len(jobs), thread_name_prefix="standby") as pool:
            futs = [pool.submit(wrap, job) for job in jobs]
            wait(futs)
        for fut in futs:
            exc = fut.exception()
            if isinstance(exc, Abort):
                emit_log("WARN", "boot", "待机位运动已中止")
                return
        if errors:
            raise errors[0]
    except Abort:
        emit_log("WARN", "boot", "待机位运动已中止")
    except Exception as exc:  # noqa: BLE001
        emit_log("ERROR", "boot", f"待机位运动失败: {exc}")


def _apply_boot_standby(steps: list[dict], *, move_now: bool) -> None:
    """把启动流程双手目标 / 腰 goto 同步到 yaml；空闲时立刻让手臂走过去。"""
    global _standby_reload_pending, _standby_move_gen
    info = boot_standby_from_steps(steps)
    left = info.get("left")
    right = info.get("right")
    waist_pose = boot_waist_from_steps(steps)
    changed = []
    if left or right:
        changed.extend(grasp_params_mod.write_standby_poses(left, right))
    if waist_pose and grasp_params_mod.write_layer3_home(waist_pose):
        changed.append("layer3_home")
    if not changed and not (left or right):
        return
    if runner.is_busy():
        if changed:
            _standby_reload_pending = True
            emit_log("INFO", "boot", "归位已写入 yaml，当前序列跑完后生效")
        return
    if changed:
        robot.reload_grasp_config()
        bits = []
        if "layer3_home" in changed and waist_pose:
            bits.append(
                f"腰=({waist_pose['x']:.3f},{waist_pose['y']:.3f},{waist_pose['z']:.3f})"
            )
        if right:
            bits.append(f"右=({right['x']:.3f},{right['y']:.3f},{right['z']:.3f})")
        if left:
            bits.append(f"左=({left['x']:.3f},{left['y']:.3f},{left['z']:.3f})")
        emit_log("INFO", "boot", "现用归位已同步 yaml（旧位姿保留为 *_old） " + " ".join(bits))
    if not move_now or not changed:
        return
    if not left and not right:
        return
    _standby_move_gen += 1
    gen = _standby_move_gen
    speed = float(info.get("speed") or 0.2)
    emit_log("INFO", "boot", "双臂立刻走到网页待机位")
    threading.Thread(
        target=_move_standby_now,
        args=(left, right, speed, gen),
        name="boot-standby-move",
        daemon=True,
    ).start()


@app.get("/")
def index():
    return send_from_directory(STATIC, "index.html")


@app.get("/api/meta")
def meta():
    return jsonify(
        {
            "library": LIBRARY,
            "chassis": "disabled",
            "mode": robot.mode_name(),
            "project": str(PROJECT_ROOT),
            "approach_paths": approach_paths_mod.list_names(),
        }
    )


@app.get("/api/state")
def get_state():
    with _state_lock:
        snap = dict(_state)
    with _seq_lock:
        snap["steps"] = list(_steps)
        snap["program_name"] = _program_name
    with _boot_lock:
        snap["boot_steps"] = list(_boot_steps)
    return jsonify(snap)


@app.get("/api/logs")
def get_logs():
    with _hist_lock:
        return jsonify({"logs": list(_log_hist)})


@app.post("/api/logs/clear")
def clear_logs():
    with _hist_lock:
        _log_hist.clear()
    _broadcast("logs_cleared", {})
    return jsonify({"ok": True})


@app.get("/api/events")
def sse():
    q: queue.Queue = queue.Queue(maxsize=200)
    with _sse_lock:
        _sse_queues.append(q)

    def gen():
        yield "event: hello\ndata: {}\n\n"
        with _state_lock:
            snap = dict(_state)
        yield f"event: state\ndata: {json.dumps(snap, ensure_ascii=False)}\n\n"
        try:
            while True:
                try:
                    payload = q.get(timeout=12)
                    yield f"data: {payload}\n\n"
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            with _sse_lock:
                if q in _sse_queues:
                    _sse_queues.remove(q)

    return Response(gen(), mimetype="text/event-stream", headers={"Cache-Control": "no-cache"})


def _set_steps(steps: list[dict], name: str | None = None) -> list[dict]:
    global _steps, _program_name
    clean = [canonicalize_step(s) for s in steps]
    with _seq_lock:
        _steps = clean
        if name:
            _program_name = name
        current_name = _program_name
    _broadcast("sequence", {"steps": clean, "program_name": current_name})
    return clean


def _persist_boot(steps: list[dict]) -> None:
    doc = program_document("boot_init", steps)
    BOOT_FILE.write_text(json.dumps(doc, ensure_ascii=False, indent=2), encoding="utf-8")


def _set_boot_steps(steps: list[dict], *, persist: bool = True) -> list[dict]:
    global _boot_steps
    clean = [canonicalize_step(s) for s in steps]
    with _boot_lock:
        _boot_steps = clean
    if persist:
        _persist_boot(clean)
    _broadcast("boot_sequence", {"steps": clean})
    return clean


def _load_boot_file() -> list[dict]:
    if BOOT_FILE.is_file():
        try:
            doc = json.loads(BOOT_FILE.read_text(encoding="utf-8"))
            return [canonicalize_step(s) for s in (doc.get("steps") or [])]
        except (ValueError, json.JSONDecodeError) as exc:
            emit_log("WARN", "boot", f"启动流程文件损坏，改用默认: {exc}")
    return default_boot_steps()


@app.post("/api/sequence")
def set_sequence():
    body = request.get_json(force=True, silent=True) or {}
    try:
        clean = _set_steps(body.get("steps") or [], body.get("name"))
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    return jsonify({"ok": True, "steps": clean})


def _seed_from_hw(incoming: dict) -> dict:
    incoming = dict(incoming)
    try:
        hw = robot.hw_snapshot()
    except Exception:  # noqa: BLE001
        hw = {}
    if incoming.get("kind") == "waist" and incoming.get("mode") == "goto" and hw.get("waist_tcp"):
        incoming["pose"] = dict(hw["waist_tcp"])
    if incoming.get("kind") == "arm":
        tcp = hw.get("tcp") or {}
        if incoming.get("left") and tcp.get("left"):
            incoming["left"] = dict(tcp["left"])
        if incoming.get("right") and tcp.get("right"):
            incoming["right"] = dict(tcp["right"])
    return incoming


@app.post("/api/sequence/add")
def add_step():
    body = request.get_json(force=True, silent=True) or {}
    incoming = _seed_from_hw(dict(body.get("step") or {}))
    target_id = body.get("target_id")
    with _seq_lock:
        current = list(_steps)
    try:
        new_steps, msg, merged = merge_into_sequence(current, incoming, target_id=target_id)
        clean = _set_steps(new_steps)
    except ValueError as exc:
        emit_log("WARN", "seq", str(exc))
        return jsonify({"ok": False, "error": str(exc)}), 400
    emit_log("INFO", "seq", msg)
    return jsonify({"ok": True, "steps": clean, "message": msg, "merged": merged})


@app.post("/api/sequence/validate")
def validate():
    body = request.get_json(force=True, silent=True) or {}
    steps = body.get("steps")
    with _seq_lock:
        use = steps if steps is not None else list(_steps)
    errors = validate_program(use)
    return jsonify({"ok": not errors, "errors": errors})


@app.post("/api/sequence/run")
def run_seq():
    body = request.get_json(force=True, silent=True) or {}
    with _seq_lock:
        steps = list(_steps)
    loops = int(body.get("loops") if body.get("loops") is not None else 1)
    from_id = body.get("from_id")
    try:
        _flush_standby_reload()
        runner.start(steps, loops=loops, from_id=from_id)
    except (ValueError, RuntimeError) as exc:
        emit_log("ERROR", "run", str(exc))
        return jsonify({"ok": False, "error": str(exc)}), 400
    emit_log("INFO", "run", "开始运行序列")
    return jsonify({"ok": True})


@app.post("/api/sequence/stop")
def stop_seq():
    runner.request_stop()
    return jsonify({"ok": True})


@app.get("/api/boot_sequence")
def get_boot_sequence():
    with _boot_lock:
        steps = list(_boot_steps)
    return jsonify({"ok": True, "steps": steps})


@app.post("/api/boot_sequence")
def set_boot_sequence():
    body = request.get_json(force=True, silent=True) or {}
    try:
        clean = _set_boot_steps(body.get("steps") or [])
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    _apply_boot_standby(clean, move_now=True)
    return jsonify({"ok": True, "steps": clean})


@app.post("/api/boot_sequence/add")
def add_boot_step():
    body = request.get_json(force=True, silent=True) or {}
    incoming = _seed_from_hw(dict(body.get("step") or {}))
    target_id = body.get("target_id")
    with _boot_lock:
        current = list(_boot_steps)
    try:
        new_steps, msg, merged = merge_into_sequence(current, incoming, target_id=target_id)
        clean = _set_boot_steps(new_steps)
    except ValueError as exc:
        emit_log("WARN", "boot", str(exc))
        return jsonify({"ok": False, "error": str(exc)}), 400
    _apply_boot_standby(clean, move_now=True)
    emit_log("INFO", "boot", msg)
    return jsonify({"ok": True, "steps": clean, "message": msg, "merged": merged})


@app.post("/api/boot_sequence/run")
def run_boot_sequence():
    with _boot_lock:
        steps = list(_boot_steps)
    try:
        _flush_standby_reload()
        runner.start(steps, loops=1)
    except (ValueError, RuntimeError) as exc:
        emit_log("ERROR", "boot", str(exc))
        return jsonify({"ok": False, "error": str(exc)}), 400
    emit_log("INFO", "boot", "开始试跑启动初始动作")
    return jsonify({"ok": True})


@app.post("/api/boot_sequence/reset")
def reset_boot_sequence():
    clean = _set_boot_steps(default_boot_steps())
    _apply_boot_standby(clean, move_now=True)
    emit_log("INFO", "boot", "已恢复默认启动初始动作")
    return jsonify({"ok": True, "steps": clean})


@app.get("/api/programs")
def list_programs():
    names = sorted(p.stem for p in SEQ_DIR.glob("*.json") if not p.stem.startswith("_"))
    return jsonify({"programs": names})


@app.post("/api/programs")
def save_program():
    body = request.get_json(force=True, silent=True) or {}
    name = (body.get("name") or "").strip()
    if not name:
        return jsonify({"ok": False, "error": "需要名称"}), 400
    safe = "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in name)
    with _seq_lock:
        steps = list(_steps)
    doc = program_document(safe, steps)
    path = SEQ_DIR / f"{safe}.json"
    path.write_text(json.dumps(doc, ensure_ascii=False, indent=2), encoding="utf-8")
    _set_steps(steps, safe)
    emit_log("INFO", "io", f"已保存 {path.name}")
    return jsonify({"ok": True, "file": path.name})


@app.get("/api/programs/<name>")
def load_program(name: str):
    safe = Path(name).stem
    if safe.startswith("_"):
        return jsonify({"ok": False, "error": "文件不存在"}), 404
    path = SEQ_DIR / f"{safe}.json"
    if not path.exists():
        return jsonify({"ok": False, "error": "文件不存在"}), 404
    doc = json.loads(path.read_text(encoding="utf-8"))
    try:
        clean = _set_steps(doc.get("steps") or [], doc.get("name") or safe)
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    emit_log("INFO", "io", f"已加载 {path.name}（{len(clean)} 步）")
    return jsonify({"ok": True, "doc": program_document(doc.get("name") or safe, clean)})


@app.post("/api/manual")
def manual():
    if runner.is_busy():
        return jsonify({"ok": False, "error": "序列运行中，仅允许停止"}), 409
    body = request.get_json(force=True, silent=True) or {}
    target = body.get("target")
    try:
        if target == "arm":
            side = body.get("side")
            if side not in ("left", "right"):
                raise ValueError("side 必须是 left/right")
            speed = float(body.get("speed") or 0.2)
            pose = canonicalize_step({"kind": "arm", side: body.get("pose"), "speed": speed})[side]
            robot.move_arm(side, pose, speed)
        elif target == "gripper":
            robot.gripper(body["side"], body["cmd"])
        elif target == "waist_rotate":
            robot.waist_rotate(float(body.get("angle_deg") or 0), float(body.get("speed_deg_s") or 30))
        elif target == "waist_fold":
            robot.waist_fold(float(body.get("angle_deg") or 0), float(body.get("speed_deg_s") or 30))
        elif target == "waist_lift":
            robot.waist_lift(float(body.get("delta_m") or 0), float(body.get("speed") or 0.1))
        elif target == "waist_advance":
            robot.waist_advance(float(body.get("delta_m") or 0), float(body.get("speed") or 0.1))
        elif target == "waist_goto":
            pose = canonicalize_step({"kind": "waist", "mode": "goto", "pose": body.get("pose"), "speed": body.get("speed") or 0.1})["pose"]
            robot.waist_goto(pose, float(body.get("speed") or 0.1))
        elif target == "head":
            robot.set_head(float(body.get("m0") or 0), float(body.get("m1") or 0), float(body.get("m2") or 0))
        elif target == "home":
            robot.home()
        elif target == "chassis":
            return jsonify({"ok": False, "error": "底盘已禁用"}), 400
        else:
            return jsonify({"ok": False, "error": f"未知手动目标 {target}"}), 400
    except Exception as exc:  # noqa: BLE001
        emit_log("ERROR", "manual", str(exc))
        return jsonify({"ok": False, "error": str(exc)}), 400
    on_state(runner.snapshot())
    return jsonify({"ok": True, "robot": robot.snapshot()})


@app.get("/api/hw/snapshot")
def hw_snapshot():
    try:
        return jsonify({"ok": True, **robot.hw_snapshot()})
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 503


@app.get("/api/grasp_params")
def get_grasp_params():
    return jsonify({"ok": True, "fields": grasp_params_mod.schema(), "values": grasp_params_mod.read_all()})


@app.post("/api/grasp_params")
def set_grasp_params():
    body = request.get_json(force=True, silent=True) or {}
    values = body.get("values") if isinstance(body.get("values"), dict) else body
    try:
        changed = grasp_params_mod.write_values(values or {})
        robot.reload_grasp_config()
        emit_log("INFO", "cfg", "已保存抓取参数 " + (", ".join(changed) if changed else "(无改动)"))
        return jsonify({"ok": True, "changed": changed, "values": grasp_params_mod.read_all()})
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.get("/api/approach_paths")
def get_approach_paths():
    doc = approach_paths_mod.load_doc()
    return jsonify({"ok": True, "paths": doc["paths"], "names": approach_paths_mod.list_names()})


@app.post("/api/approach_paths")
def save_approach_path():
    body = request.get_json(force=True, silent=True) or {}
    name = str(body.get("name") or "").strip()
    if not name:
        return jsonify({"ok": False, "error": "缺少路径名"}), 400
    try:
        path = approach_paths_mod.upsert_path(name, body.get("path") or body)
        emit_log("INFO", "path", f"已保存接近路径 {name}（{len(path.get('waypoints') or [])} 点）")
        return jsonify({"ok": True, "name": name, "path": path, "names": approach_paths_mod.list_names()})
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/approach_paths/delete")
def delete_approach_path():
    body = request.get_json(force=True, silent=True) or {}
    name = str(body.get("name") or "").strip()
    try:
        approach_paths_mod.delete_path(name)
        emit_log("INFO", "path", f"已删除接近路径 {name}")
        return jsonify({"ok": True, "names": approach_paths_mod.list_names(), "paths": approach_paths_mod.load_doc()["paths"]})
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/approach_paths/record")
def record_approach_waypoint():
    if runner.is_busy():
        return jsonify({"ok": False, "error": "序列正在跑，先 STOP 再示教"}), 409
    body = request.get_json(force=True, silent=True) or {}
    name = str(body.get("name") or "").strip()
    if not name:
        return jsonify({"ok": False, "error": "缺少路径名"}), 400
    try:
        path = approach_paths_mod.get_path(name) or approach_paths_mod.upsert_path(name, body.get("meta") or {})
        arm = str(body.get("arm") or path.get("arm") or "left")
        marker_id = int(body.get("marker_id") if body.get("marker_id") not in (None, "") else path.get("marker_id") or 0)
        marker_id_b = int(body.get("marker_id_b") if body.get("marker_id_b") not in (None, "") else path.get("marker_id_b") or 0)
        rec = robot.path_record(arm, marker_id, marker_id_b)
        wp_name = str(body.get("waypoint_name") or "").strip() or f"p{len(path['waypoints']) + 1}"
        rel = rec.get("rel") or {}
        wp = {
            "name": wp_name,
            "x": float(rel.get("x") or 0),
            "y": float(rel.get("y") or 0),
            "z": float(rel.get("z") or 0),
            "rx": float(rel.get("rx") or 0),
            "ry": float(rel.get("ry") or 0),
            "rz": float(rel.get("rz") or 0),
        }
        meta = {
            "arm": arm,
            "marker_id": int(rec.get("marker_id") or marker_id or 0),
            "marker_id_b": int(rec.get("marker_id_b") or marker_id_b or 0),
        }
        if rec.get("have_b") and rec.get("dual_rel") and not path.get("dual_rel"):
            meta["dual_rel"] = rec["dual_rel"]
        path = dict(path)
        path.update(meta)
        path["waypoints"] = list(path.get("waypoints") or []) + [wp]
        saved = approach_paths_mod.upsert_path(name, path)
        emit_log("INFO", "path", f"{name} 记录路径点 {wp_name} 主码={rec.get('marker_id')}")
        return jsonify({"ok": True, "name": name, "path": saved, "record": rec, "names": approach_paths_mod.list_names()})
    except Exception as exc:  # noqa: BLE001
        emit_log("ERROR", "path", str(exc))
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/approach_paths/detect")
def detect_approach_markers():
    if runner.is_busy():
        return jsonify({"ok": False, "error": "序列正在跑，先 STOP 再拍码"}), 409
    body = request.get_json(force=True, silent=True) or {}
    arm = str(body.get("arm") or "left")
    marker_id = int(body.get("marker_id") or 0)
    marker_id_b = int(body.get("marker_id_b") or 0)
    try:
        rec = robot.path_record(arm, marker_id, marker_id_b)
        emit_log("INFO", "path", f"拍码 主={rec.get('marker_id')} 副={rec.get('marker_id_b')}")
        return jsonify({"ok": True, "record": rec})
    except Exception as exc:  # noqa: BLE001
        emit_log("ERROR", "path", str(exc))
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/approach_paths/replay")
def replay_approach_path():
    if runner.is_busy():
        return jsonify({"ok": False, "error": "序列正在跑，先 STOP 再试走"}), 409
    body = request.get_json(force=True, silent=True) or {}
    name = str(body.get("name") or "").strip()
    try:
        path = approach_paths_mod.get_path(name)
        if path is None:
            raise ValueError(f"没有接近路径 {name}")
        index = body.get("index")
        extras = approach_paths_mod.rpc_fields(
            name,
            index=None if index in (None, "") else int(index),
            include_target=body.get("include_target"),
        )
        side = str(body.get("side") or path.get("side") or "left")
        arm = str(body.get("arm") or path.get("arm") or "left")
        if side == "above":
            robot.aruco_above(hand="empty", arm=arm, extras=extras)
        else:
            robot.conveyor_goto(side=side, hand="empty", arm=arm, extras=extras)
        emit_log("INFO", "path", f"试走 {name}" + (f" 第{int(index)+1}点" if index not in (None, "") else " 全程"))
        return jsonify({"ok": True})
    except Exception as exc:  # noqa: BLE001
        emit_log("ERROR", "path", str(exc))
        return jsonify({"ok": False, "error": str(exc)}), 400


def _latest_debug_jpg(folder: Path):
    if not folder.is_dir():
        return None
    files = [p for p in folder.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png")]
    if not files:
        return None
    files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0]


@app.get("/api/debug_vis/latest")
def debug_vis_latest():
    out = {}
    for key, sub in (("head", "head"), ("right_hand", "right_hand"), ("left_hand", "left_hand")):
        proc = _latest_debug_jpg(DEBUG_VIS / sub)
        if proc:
            mt = int(proc.stat().st_mtime)
            out[key] = f"/debug_vis/{sub}/{proc.name}?t={mt}"
            orig = DEBUG_VIS / "original" / sub / proc.name
            if orig.is_file():
                out[key + "_orig"] = f"/debug_vis/original/{sub}/{orig.name}?t={int(orig.stat().st_mtime)}"
    return jsonify({"ok": True, "images": out})


@app.get("/debug_vis/<path:rel>")
def debug_vis_file(rel: str):
    root = DEBUG_VIS.resolve()
    target = (DEBUG_VIS / rel).resolve()
    if root not in target.parents and target != root:
        return jsonify({"ok": False, "error": "forbidden"}), 403
    if not target.is_file():
        return jsonify({"ok": False, "error": "not found"}), 404
    return send_from_directory(target.parent, target.name)


def _seed_example() -> None:
    demo = SEQ_DIR / "demo_sim.json"
    if demo.exists():
        return
    doc = program_document(
        "demo_sim",
        [
            {"kind": "home"},
            {"kind": "gripper", "left": "open", "right": "open"},
            {
                "kind": "arm",
                "left": {"x": 0.50, "y": 0.30, "z": -0.20, "rx": 90, "ry": 0, "rz": 0},
                "right": {"x": 0.50, "y": -0.30, "z": -0.20, "rx": -90, "ry": 0, "rz": 0},
                "speed": 0.2,
                "note": "示例：双臂同时靠近",
            },
            {"kind": "sleep", "seconds": 0.5},
            {"kind": "gripper", "left": "close", "right": "close"},
            {"kind": "home"},
        ],
    )
    demo.write_text(json.dumps(doc, ensure_ascii=False, indent=2), encoding="utf-8")


def _run_boot_after_listen() -> None:
    time.sleep(1.2)
    with _boot_lock:
        steps = list(_boot_steps)
    if not steps:
        emit_log("WARN", "boot", "启动初始动作为空，跳过")
        return
    try:
        _flush_standby_reload()
        runner.start(steps, loops=1)
        emit_log("INFO", "boot", "正在执行启动初始动作（可在网页修改）")
    except Exception as exc:  # noqa: BLE001
        emit_log("ERROR", "boot", f"启动初始动作未能开始: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--mode", default=os.environ.get("ORCH_MODE", "hw"), choices=("sim", "hw", "hardware"))
    args = parser.parse_args()
    _seed_example()
    existed = BOOT_FILE.is_file()
    boot_steps = _set_boot_steps(_load_boot_file(), persist=not existed)
    _apply_boot_standby(boot_steps, move_now=False)
    emit_log("INFO", "sys", f"编排台启动  mode={robot.mode_name()}  chassis=disabled")
    emit_log("INFO", "sys", f"打开 http://127.0.0.1:{args.port}/")
    emit_log("INFO", "boot", f"启动初始动作 {len(_boot_steps)} 步，来自 {BOOT_FILE.name if existed else '默认'}")
    threading.Thread(target=_run_boot_after_listen, name="boot-init", daemon=True).start()
    app.run(host=args.host, port=args.port, threaded=True, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
