#!/usr/bin/env python3
"""T170C robot-hosted debug web server.

The browser talks only to this process. Hardware remains exclusively owned by
the localhost C++ daemon (t170_factory on 127.0.0.1:8099).
"""

from __future__ import annotations

import argparse
import hmac
import json
import mimetypes
import os
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import yaml


# 前端 app.js 里有同一个数，对不上时页面提示重启 server.py 并强制刷新。
WEB_API_VERSION = 9
UI_VERSION = "2.2.0"
LIVE_SHM = Path("/dev/shm/t170-hand-live")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from t170_common.paths import load_paths
from t170_common.config_workspace import ConfigWorkspace, workspace_root
from web_debug.profiles import ProfileStore
from web_debug.actions import catalog as action_catalog
from t170_common.web_access import persistent_token
from t170_common.maintenance_workcells import load_catalog as load_workcells, validate_catalog, STATION_FIELDS

RUNTIME_PATHS = load_paths()
PROJECT_ROOT = RUNTIME_PATHS.root
STATIC_ROOT = Path(__file__).resolve().parent / "static"
WORKSPACE = ConfigWorkspace(workspace_root(PROJECT_ROOT))
PROFILE_STORE = ProfileStore(WORKSPACE.root)
CONFIG_FILES = {
    "motion": RUNTIME_PATHS.config_dir / "effective_params.yaml",
    "tray_code": PROJECT_ROOT / "board_yf100_aruco" / "config.yaml",
    "belt_code": PROJECT_ROOT / "board_belt_aruco" / "config.yaml",
    "hand_eye": PROJECT_ROOT / "free_calib" / "config" / "camera_to_head_connector_result.yaml",
    "head_geom": PROJECT_ROOT / "free_calib" / "config" / "head_params.yaml",
    "hand_eye_fallback": RUNTIME_PATHS.config_dir / "camera_to_base_result.yaml",
}
CONFIG_NOTES = {
    "motion": "修改永久保存到本机 quick_params.yaml；空闲时热更新，需重启的改动在下次启动加载。",
    "tray_code": "料盘二维码。保存后要重启 t170_factory 才生效。",
    "belt_code": "传送带二维码。保存后要重启 t170_factory 才生效。",
    "hand_eye": "相机到头部的平移。保存后要重启 t170_factory。程序读矩阵，平移会同时写进矩阵。",
    "head_geom": "头部离基座的高度。保存后要重启 t170_factory。",
    "hand_eye_fallback": "手眼失败时的回退。这一页仍是原文，保存后要重启。",
}


def config_path(file_id):
    if (WORKSPACE.root / "config/deployment.yaml").is_file():
        return WORKSPACE.source_file(file_id)
    return CONFIG_FILES[file_id]


def apply_persistent(state, file_id=None, content=None, render=None):
    def request(payload, timeout=30):
        try: return state.robot_request(payload, timeout=timeout)
        except RobotClientError as error: raise ConnectionError(str(error)) from error
    if file_id is None: return WORKSPACE.apply(request)
    if render is not None: return WORKSPACE.edit(file_id, render, request)
    return WORKSPACE.save(file_id, content, request)


def maintenance_workcells_view(state):
    path = WORKSPACE.source_file('maintenance_workcells')
    catalog = load_workcells(path)
    result = dict(ok=True, path=str(path), selected_group=catalog['selected_group'],
                  groups=catalog['groups'], active_group=None, editable=False)
    try:
        live = state.robot_request({'cmd': 'ping'}, timeout=3)
        result['editable'] = live.get('ok') is True and live.get('mode') == 'maintenance'
        directory = Path(str(live.get('config_dir', '')))
        if directory.is_absolute():
            # C++ publishes this snapshot atomically with its in-memory commit.
            # Do not present a saved-but-rejected selection as active.
            params = yaml.safe_load((directory/'effective_params.yaml').read_text())
            actual = params.get('chassis', {})
            result['active_stations'] = {key: actual.get(key) for key in STATION_FIELDS}
            result['active_group'] = next((row['group'] for row in catalog['groups']
                if all(actual.get(key) == row[key] for key in STATION_FIELDS)), None)
    except (RobotClientError, OSError):
        pass
    return result


def select_maintenance_workcell(state, group):
    if type(group) is not int:
        raise ValueError('请选择有效的机床组编号')

    def request(payload, timeout=30):
        try:
            reply = state.robot_request(payload, timeout=timeout)
        except RobotClientError as error:
            if payload['cmd'] == 'ping':
                raise ValueError('请先启动维护服务，再切换机床组') from error
            # An uncertain reload may already have committed: never blindly
            # roll back its source; preserve the existing unconfirmed outcome.
            raise ConnectionError(str(error)) from error
        if payload['cmd'] == 'ping':
            if not reply.get('ok') or reply.get('mode') != 'maintenance':
                raise ValueError('机床组选择仅用于维护模式，生产任务使用调度站点')
        elif not reply.get('ok'):
            # _save restores the source under its config lock on rejection.
            raise ValueError(reply.get('error') or '机床组尚未切换，请等待动作或标定结束')
        return reply

    request({'cmd': 'ping'}, timeout=3)
    calibration = state.robot_request({'cmd': 'calibration_status'}, timeout=3)
    if not calibration.get('ok') or calibration.get('active'):
        raise ValueError('请先结束标定，再切换机床组')

    def render(text):
        catalog = yaml.safe_load(text)
        catalog['selected_group'] = group
        validate_catalog(catalog)
        return yaml.safe_dump(catalog, sort_keys=False, allow_unicode=True)

    result = WORKSPACE.edit('maintenance_workcells', render, request)
    result['workcells'] = maintenance_workcells_view(state)
    return result
MANUAL_TASKS = {
    "load": ["vision_grasp_once", "belt"],
    "transfer": ["belt_grasp", "belt2"],
    "unload": ["belt2_grasp", "tray2_auto"],
}
MANUAL_LABELS = {"load": "上料", "transfer": "转运", "unload": "下料"}

# 只暴露平时会改的数。path 用点号，列表用下标。保存时按行替换，注释保留。
from t170_common.parameter_catalog import FIELD_GROUPS


def _yaml_lookup(node: Any, parts: list[str]) -> Any:
    cur = node
    for part in parts:
        if isinstance(cur, dict):
            if part not in cur:
                return None
            cur = cur[part]
        elif isinstance(cur, list) and part.isdigit():
            index = int(part)
            if index < 0 or index >= len(cur):
                return None
            cur = cur[index]
        else:
            return None
    return cur


def _replace_yaml_scalar(text: str, parts: list[str], value_text: str) -> str:
    # Node spans support both block and flow collections and preserve comments,
    # quoting and every unrelated value instead of guessing from indentation.
    node = yaml.compose(text)
    for part in parts:
        if isinstance(node, yaml.MappingNode):
            matches = [value for key, value in node.value if key.value == part]
            if len(matches) != 1: raise KeyError(".".join(parts))
            node = matches[0]
        elif isinstance(node, yaml.SequenceNode) and part.isdigit() and int(part) < len(node.value):
            node = node.value[int(part)]
        else: raise KeyError(".".join(parts))
    if not isinstance(node, yaml.ScalarNode): raise KeyError(".".join(parts))
    return text[:node.start_mark.index] + value_text + text[node.end_mark.index:]


def _replace_yaml_value(text: str, path: str, value_text: str) -> str:
    return _replace_yaml_scalar(text, path.split("."), value_text)


def _sync_hand_eye_matrix(text: str, xyz: list[float]) -> str:
    """平移写在 b. translation 和 a. matrix 的最后一列。旋转不动。"""
    matrix = yaml.safe_load(text).get("a. matrix")
    if not isinstance(matrix, list) or len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise KeyError("a. matrix")
    for row, value in enumerate(xyz):
        text = _replace_yaml_scalar(text, ["a. matrix", str(row), "3"], repr(float(value)))
    return text


def read_field_groups(file_id: str) -> list[dict[str, Any]]:
    path = config_path(file_id)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    groups = []
    for title, fields in FIELD_GROUPS[file_id]:
        rows = []
        for key, label, unit in fields:
            if file_id == "hand_eye":
                vec = data.get("b. translation_xyz_mm") or [None, None, None]
                value = vec[{"tx": 0, "ty": 1, "tz": 2}[key]]
            else:
                value = _yaml_lookup(data, key.split("."))
            rows.append({"key": key, "label": label, "unit": unit, "value": value})
        groups.append({"title": title, "fields": rows})
    return groups


def render_field_groups(file_id: str, updates: dict[str, str], text: str | None = None) -> str:
    if text is None: text = config_path(file_id).read_text(encoding="utf-8")
    allowed = {key for _title, fields in FIELD_GROUPS[file_id] for key, _label, _unit in fields}
    unknown = [key for key in updates if key not in allowed]
    if unknown:
        raise KeyError("、".join(unknown))
    if file_id == "hand_eye":
        data = yaml.safe_load(text) or {}
        vec = [float(v) for v in (data.get("b. translation_xyz_mm") or [0, 0, 0])]
        slot = {"tx": 0, "ty": 1, "tz": 2}
        for key, raw in updates.items():
            vec[slot[key]] = float(raw)
        for index, value in enumerate(vec):
            text = _replace_yaml_scalar(text, ["b. translation_xyz_mm", str(index)], repr(value))
        text = _sync_hand_eye_matrix(text, vec)
    else:
        for key, raw in updates.items():
            text = _replace_yaml_value(text, key, str(raw).strip())
    return text

PICTURE_ROOT = PROJECT_ROOT / "debug_picture"
RUNTIME_ROOT = RUNTIME_PATHS.log_dir / "web"
CAMERA_SLOTS = ("head", "right", "left")
ALLOWED_COMMANDS = {
    "ping",
    "snapshot",
    "reload_config",
    "aruco_detect",
    "vision_grasp",
    "vision_grasp_once",
    "vision_servo_dual_once",
    "dispatch",
    "charge",
    "home",
    "grasp_ready",
    "grasp_ready1",
    "grasp_ready2",
    "grasp_ready3",
    "grasp_ready6",
    "tray2ready",
    "tray2ready1",
    "tray2ready2",
    "tray2ready3",
    "tray2ready6",
    "tray2",
    "tray2_auto",
    "tray2_place",
    "waist1",
    "waist2",
    "waist_cycle",
    "belt",
    "belt_ready",
    "belt_place",
    "belt_grasp_rpy",
    "belt_grasp",
    "belt2",
    "belt2_ready",
    "belt2_place",
    "belt2_grasp_rpy",
    "belt2_grasp",
    "waist_jog",
    "wave",
    "bow",
    "welcome",
    "wave_ready",
    "detect_tray_holes",
    "abort",
    "dispatch_exit",
}
DANGEROUS_COMMANDS = ALLOWED_COMMANDS - {
    "ping",
    "snapshot",
    "reload_config",
    "aruco_detect",
    "detect_tray_holes",
    "abort",
    "dispatch_exit",
}


class RobotClientError(RuntimeError):
    pass


def command_result(cmd: str, response: dict[str, Any]) -> dict[str, Any]:
    """Keep business failure distinct from a successful HTTP/RPC exchange.

    Also reject the old daemon's `ok:true, code:2, grasped_count:0` response,
    so updating the Web process first cannot silently report a failed grasp.
    """
    result = dict(response)
    task_counts = {"vision_grasp_once": "grasped_count", "vision_servo_dual_once": "grasped_count",
                   "vision_grasp": "grasped_count", "tray2": "placed_count",
                   "tray2_place": "placed_count", "tray2_auto": "placed_count"}
    count_key = task_counts.get(cmd)
    if count_key and result.get("ok") is True:
        code, count = result.get("code"), result.get(count_key)
        if code != 0 or not isinstance(count, (int, float)) or count <= 0:
            result["ok"] = False
            result["status"] = "partial" if isinstance(count, (int, float)) and count > 0 else "no_target" if code == 1 else "failed"
            result["error"] = result.get("error") or f"{cmd} 未完成：code={code}，确认数量={count}"
    result["ok"] = result.get("ok") is True
    if not result["ok"]:
        result.setdefault("status", "failed")
        result["error"] = result.get("error") or result.get("message") or f"{cmd} 执行失败"
    return result


class AppState:
    def __init__(self, args: argparse.Namespace) -> None:
        self.robot_host = args.robot_host
        self.robot_port = args.robot_port
        self.robot_command = args.robot_command
        self.token = args.token or persistent_token(WORKSPACE.root)
        self.token_generated = False
        self.robot_process: subprocess.Popen[str] | None = None
        self.log_lines: deque[dict[str, Any]] = deque(maxlen=3000)
        self.log_seq = 0
        self.log_lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.job_lock = threading.Lock()
        self.job: dict[str, Any] = {"running": False, "title": "", "step": "", "error": ""}
        self.last_snapshot: dict[str, Any] = {}
        RUNTIME_ROOT.mkdir(parents=True, exist_ok=True)

    def log(self, text: str, source: str = "web") -> None:
        text = text.rstrip("\r\n")
        if not text:
            return
        with self.log_lock:
            self.log_seq += 1
            self.log_lines.append(
                {
                    "seq": self.log_seq,
                    "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
                    "source": source,
                    "text": text,
                }
            )

    def logs_since(self, since: int) -> tuple[int, list[dict[str, Any]]]:
        with self.log_lock:
            return self.log_seq, [line for line in self.log_lines if line["seq"] > since]

    def robot_request(self, payload: dict[str, Any], timeout: float | None = 1800.0) -> dict[str, Any]:
        data = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
        try:
            with socket.create_connection((self.robot_host, self.robot_port), timeout=3.0) as sock:
                sock.settimeout(timeout)
                sock.sendall(data)
                chunks = bytearray()
                while not chunks.endswith(b"\n"):
                    chunk = sock.recv(65536)
                    if not chunk:
                        break
                    chunks.extend(chunk)
                    if len(chunks) > 4 * 1024 * 1024:
                        raise RobotClientError("机器人服务响应超过 4 MiB")
        except OSError as exc:
            raise RobotClientError(f"无法连接机器人服务: {exc}") from exc
        if not chunks:
            raise RobotClientError("机器人服务未返回数据")
        try:
            result = json.loads(chunks.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RobotClientError(f"机器人服务返回无效 JSON: {exc}") from exc
        if not isinstance(result, dict):
            raise RobotClientError("机器人服务响应不是 JSON 对象")
        return result

    def job_view(self) -> dict[str, Any]:
        with self.job_lock:
            return dict(self.job)

    def begin_action(self, title: str) -> bool:
        with self.job_lock:
            if self.job.get("running"):
                return False
            self.job = {"running": True, "title": title, "step": title, "error": ""}
            return True

    def end_action(self, error: str = "") -> None:
        with self.job_lock:
            self.job = {
                "running": False,
                "title": self.job.get("title", ""),
                "step": "失败" if error else "完成",
                "outcome": "failed" if error else "succeeded",
                "error": error,
            }

    def run_sequence(self, steps: list[str]) -> None:
        with self.job_lock:
            self.job["step"] = "准备"
        try:
            for task in steps:
                commands = MANUAL_TASKS[task]
                for cmd in commands:
                    with self.job_lock:
                        self.job["step"] = f"{MANUAL_LABELS[task]} · {cmd}"
                    self.log(f"手动编排 {MANUAL_LABELS[task]} → {cmd}", "sequence")
                    result = command_result(cmd, self.robot_request({"cmd": cmd}))
                    self.log(f"命令 {cmd}: {json.dumps(result, ensure_ascii=False)}", "command")
                    if not result.get("ok"):
                        raise RobotClientError(result.get("error") or f"{cmd} 失败")
            with self.job_lock:
                self.job = {"running": False, "title": "手动编排", "step": "完成", "error": ""}
        except Exception as exc:  # noqa: BLE001
            self.log(f"手动编排停止: {exc}", "sequence")
            self.end_action(str(exc))

    def robot_online(self) -> bool:
        try:
            return bool(self.robot_request({"cmd": "ping"}, timeout=3.0).get("ok"))
        except RobotClientError:
            return False

    def start_robot(self) -> bool:
        if self.robot_online():
            self.log("检测到现有 t170_factory，Web 调试台将连接该进程")
            return True
        if self.robot_process and self.robot_process.poll() is None:
            return False

        command = shlex.split(self.robot_command)
        if not command:
            raise RobotClientError("robot-command 为空")
        binary = Path(command[0])
        if not binary.is_absolute():
            binary = PROJECT_ROOT / binary
        if not binary.exists():
            raise RobotClientError(f"未找到机器人程序: {binary}")
        command[0] = str(binary)
        if shutil.which("stdbuf"):
            command = ["stdbuf", "-oL", "-eL", *command]

        log_path = RUNTIME_ROOT / "robot.log"
        log_file = log_path.open("a", encoding="utf-8")
        self.log("启动机器人服务: " + " ".join(shlex.quote(x) for x in command))
        self.robot_process = subprocess.Popen(
            command,
            cwd=PROJECT_ROOT,
            stdin=None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )

        def read_output() -> None:
            assert self.robot_process is not None
            assert self.robot_process.stdout is not None
            try:
                for line in self.robot_process.stdout:
                    log_file.write(line)
                    log_file.flush()
                    print(line, end="", flush=True)
                    self.log(line, "robot")
            finally:
                code = self.robot_process.poll()
                self.log(f"机器人服务已退出，code={code}", "robot")
                log_file.close()

        threading.Thread(target=read_output, name="robot-log", daemon=True).start()
        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline:
            if self.robot_process.poll() is not None:
                raise RobotClientError(f"机器人服务启动失败，退出码 {self.robot_process.returncode}")
            if self.robot_online():
                self.log("机器人服务初始化完成")
                return True
            time.sleep(0.5)
        raise RobotClientError("机器人服务初始化超时（120 秒）")

    def stop_managed_robot(self) -> None:
        proc = self.robot_process
        if proc is None or proc.poll() is not None:
            return
        try:
            self.robot_request({"cmd": "abort"}, timeout=2.0)
        except RobotClientError:
            pass
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()


def json_bytes(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def latest_image(slot: str, kind: str) -> Path | None:
    if slot not in CAMERA_SLOTS or kind not in {"processed", "original"}:
        return None
    folder = PICTURE_ROOT / slot if kind == "processed" else PICTURE_ROOT / "original" / slot
    if not folder.is_dir():
        return None
    candidates = [p for p in folder.glob("*.jpg") if p.is_file()]
    return max(candidates, key=lambda p: p.stat().st_mtime_ns) if candidates else None


def hand_live_preview_enabled() -> bool:
    """网页实时画面会让 t170_factory 从手相机抽帧。只读配置，进程要重启才生效。"""
    path = CONFIG_FILES["motion"]
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        cameras = data.get("cameras") or {}
        return bool(cameras.get("hand_live_preview", False))
    except (OSError, yaml.YAMLError):
        return False


def host_status() -> dict[str, Any]:
    """Read inexpensive Linux host facts without shelling out."""
    can_interfaces: dict[str, str] = {}
    for index in range(6):
        name = f"can{index}"
        state_path = Path("/sys/class/net") / name / "operstate"
        can_interfaces[name] = state_path.read_text(encoding="utf-8").strip() if state_path.is_file() else "missing"
    serial_ports = sorted(str(p) for pattern in ("ttyUSB*", "ttyCH343USB*") for p in Path("/dev").glob(pattern))
    models = []
    for model in sorted((PROJECT_ROOT / "feeding_cylindrical_parts_alg" / "models").glob("*.pt")):
        stat = model.stat()
        models.append({"name": model.name, "size_mb": round(stat.st_size / 1024 / 1024, 2), "mtime_ms": int(stat.st_mtime * 1000)})
    disk = shutil.disk_usage(PROJECT_ROOT)
    try:
        load = list(os.getloadavg())
    except (AttributeError, OSError):
        load = []
    return {
        "hostname": socket.gethostname(),
        "can_interfaces": can_interfaces,
        "serial_ports": serial_ports,
        "video_devices": len(list(Path("/dev").glob("video*"))),
        "models": models,
        "load_average": load,
        "disk_free_gb": round(disk.free / 1024**3, 1),
        "config_mtime_ms": int(CONFIG_FILES["motion"].stat().st_mtime * 1000) if CONFIG_FILES["motion"].is_file() else 0,
    }


class DebugHandler(BaseHTTPRequestHandler):
    server_version = "T170DebugWeb/1.1.0"

    @property
    def state(self) -> AppState:
        return self.server.app_state  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        # 浏览器首次 URL 和旧图片链接可能带 token；日志中必须脱敏。
        message = fmt % args
        if "/api/image/live" in message or "/api/stream/live" in message:
            return
        if "token=" in message:
            prefix, _, tail = message.partition("token=")
            suffix = ""
            for separator in ("&", " "):
                if separator in tail:
                    _, suffix = tail.split(separator, 1)
                    suffix = separator + suffix
                    break
            message = prefix + "token=<redacted>" + suffix
        self.state.log(message, "http")

    def _send(self, status: int, body: bytes, content_type: str, cache: bool = False) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; img-src 'self' blob:; style-src 'self' 'unsafe-inline'; "
            "script-src 'self'; connect-src 'self'; object-src 'none'; base-uri 'none'",
        )
        self.send_header("Cache-Control", "public, max-age=3600" if cache else "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status: int, value: Any) -> None:
        self._send(status, json_bytes(value), "application/json; charset=utf-8")

    def _parse_json(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("无效 Content-Length") from exc
        if length <= 0 or length > 512 * 1024:
            raise ValueError("请求正文为空或过大")
        try:
            value = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"无效 JSON: {exc}") from exc
        if not isinstance(value, dict):
            raise ValueError("JSON 必须是对象")
        return value

    def _authorized(self, parsed: Any) -> bool:
        supplied = self.headers.get("X-T170-Token", "")
        if not supplied:
            supplied = parse_qs(parsed.query).get("token", [""])[0]
        return bool(supplied) and hmac.compare_digest(supplied, self.state.token)

    def _require_auth(self, parsed: Any) -> bool:
        if self._authorized(parsed):
            return True
        self._json(HTTPStatus.UNAUTHORIZED, {"ok": False, "error": "访问令牌无效"})
        return False

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path.startswith("/api/") and not self._require_auth(parsed):
            return
        try:
            if parsed.path == '/api/maintenance-workcells':
                self._json(HTTPStatus.OK, maintenance_workcells_view(self.state))
                return
            if parsed.path == "/api/calibration":
                self._json(HTTPStatus.OK, self.state.robot_request({"cmd": "calibration_status"}, timeout=3.0))
                return
            if parsed.path == "/api/calibration/image":
                query = parse_qs(parsed.query)
                current = self.state.robot_request({"cmd": "calibration_status"}, timeout=3.0)
                head = query.get("kind", ["hand"])[0] == "head"
                if (query.get("session", [""])[0] != current.get("session") or
                        query.get("revision", [""])[0] != str(current.get("head_image_revision" if head else "image_revision"))):
                    self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "标定图像已更新"})
                    return
                runtime = current.get("runtime_root")
                if not runtime:
                    runtime = self.state.robot_request({"cmd": "ping"}, timeout=3.0).get("runtime_root")
                if not runtime or not Path(runtime).is_absolute():
                    raise ValueError("机器人未提供有效运行目录，请更新并重启机器人服务")
                root = Path(runtime).resolve(strict=True)
                name = str(current.get("head_image_path" if head else "image_path", ""))
                path = (root / name).resolve()
                path.relative_to((root / "debug_picture/calibration").resolve())
                if not name or not path.is_file():
                    self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "当前会话图像文件不存在，请重新拍摄"})
                    return
                self._send(HTTPStatus.OK, path.read_bytes(), "image/jpeg")
                return
            if parsed.path == "/api/actions":
                capabilities = self.state.robot_request({"cmd": "capabilities"}, timeout=3.0)
                if not capabilities.get("ok"):
                    raise RobotClientError(capabilities.get("error", "无法读取当前动作目录"))
                self._json(HTTPStatus.OK, action_catalog(capabilities))
                return
            if parsed.path == "/api/profiles":
                result = PROFILE_STORE.list()
                try:
                    live = self.state.robot_request({"cmd": "ping"}, timeout=3.0)
                    result["active"] = {key: live.get(key) for key in ("mode", "runtime_root", "config_dir")}
                    runtime = Path(str(live.get("runtime_root", "")))
                    record = runtime / "deployment_resolved.json"
                    if runtime.is_absolute() and record.is_file():
                        record_data = json.loads(record.read_text(encoding="utf-8"))
                        result["active"].update({key: record_data.get(key) for key in ("robot_id", "robot_model", "manifest")})
                except RobotClientError:
                    result["active"] = None
                self._json(HTTPStatus.OK, result)
                return
            if parsed.path == "/api/capabilities":
                self._json(HTTPStatus.OK, self.state.robot_request({"cmd": "capabilities"}, timeout=3.0))
                return
            if parsed.path == "/api/health":
                online = self.state.robot_online()
                process = self.state.robot_process
                self._json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        "api": WEB_API_VERSION,
                        "ui_version": UI_VERSION,
                        "robot_online": online,
                        "managed_process": process is not None,
                        "managed_process_running": process is not None and process.poll() is None,
                        "project_root": str(PROJECT_ROOT),
                        "host": host_status(),
                        "hand_live_preview": hand_live_preview_enabled(),
                    },
                )
                return
            if parsed.path == "/api/status":
                if self.state.job_view().get("running"):
                    cached = dict(self.state.last_snapshot)
                    cached["ok"] = True
                    cached["busy"] = True
                    cached["fresh_hardware"] = False
                    cached["action"] = self.state.job_view().get("step") or "执行中"
                    self._json(HTTPStatus.OK, cached)
                    return
                result = self.state.robot_request({"cmd": "snapshot"}, timeout=15.0)
                result["fresh_hardware"] = True
                result["busy"] = False
                self.state.last_snapshot = result
                self._json(HTTPStatus.OK, result)
                return
            if parsed.path == "/api/job":
                self._json(HTTPStatus.OK, {"ok": True, **self.state.job_view()})
                return
            if parsed.path == "/api/dispatch":
                result = self.state.robot_request({"cmd": "dispatch_status"}, timeout=3.0)
                if not result.get("ok") and "未知命令" in str(result.get("error", "")):
                    result["error"] = "t170_factory 是旧版本，重启它之后才有调度面板"
                result["job"] = self.state.job_view()
                self._json(HTTPStatus.OK, result)
                return
            if parsed.path == "/api/fields":
                file_id = parse_qs(parsed.query).get("file", ["motion"])[0]
                if file_id not in FIELD_GROUPS or file_id not in CONFIG_FILES:
                    self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "没有这份参数"})
                    return
                self._json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        "file": file_id,
                        "note": CONFIG_NOTES.get(file_id, ""),
                        "editable": True,
                        "groups": read_field_groups(file_id),
                    },
                )
                return
            if parsed.path == "/api/config":
                file_id = parse_qs(parsed.query).get("file", ["motion"])[0]
                path = config_path(file_id) if file_id in CONFIG_FILES else None
                if path is None or not path.is_file():
                    self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "没有这份配置"})
                    return
                self._json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        "file": file_id,
                        "path": str(path),
                        "note": CONFIG_NOTES.get(file_id, ""),
                        "content": path.read_text(encoding="utf-8"),
                    },
                )
                return
            if parsed.path == "/api/logs":
                query = parse_qs(parsed.query)
                try:
                    since = int(query.get("since", ["0"])[0])
                except ValueError:
                    since = 0
                seq, lines = self.state.logs_since(max(0, since))
                startup = RUNTIME_PATHS.log_dir / "startup.log"
                if not startup.is_file():
                    startup = PROJECT_ROOT / "startup.log"
                startup_tail = ""
                if startup.is_file():
                    with startup.open("rb") as handle:
                        handle.seek(0, os.SEEK_END)
                        size = handle.tell()
                        handle.seek(max(0, size - 64 * 1024))
                        startup_tail = handle.read().decode("utf-8", errors="replace")
                self._json(HTTPStatus.OK, {"ok": True, "seq": seq, "lines": lines, "startup_tail": startup_tail})
                return
            if parsed.path == "/api/images":
                result: dict[str, Any] = {"ok": True, "images": {}}
                for slot in CAMERA_SLOTS:
                    result["images"][slot] = {}
                    for kind in ("processed", "original"):
                        image = latest_image(slot, kind)
                        result["images"][slot][kind] = (
                            {"name": image.name, "mtime_ms": int(image.stat().st_mtime * 1000)} if image else None
                        )
                self._json(HTTPStatus.OK, result)
                return
            if parsed.path == "/api/stream/live":
                query = parse_qs(parsed.query)
                slot = query.get("slot", [""])[0]
                if slot not in {"right_hand", "left_hand"}:
                    self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "没有这只手相机"})
                    return
                self._stream_mjpeg(LIVE_SHM / f"{slot}.jpg")
                return
            if parsed.path == "/api/image/live":
                query = parse_qs(parsed.query)
                slot = query.get("slot", [""])[0]
                if slot not in {"right_hand", "left_hand"}:
                    self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "没有这只手相机"})
                    return
                image = LIVE_SHM / f"{slot}.jpg"
                if not image.is_file():
                    self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "手相机还没有画面"})
                    return
                self._send(HTTPStatus.OK, image.read_bytes(), "image/jpeg")
                return
            if parsed.path == "/api/image/latest":
                query = parse_qs(parsed.query)
                slot = query.get("slot", ["head"])[0]
                kind = query.get("kind", ["processed"])[0]
                image = latest_image(slot, kind)
                if image is None:
                    self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "尚无该相机图像"})
                    return
                self._send(HTTPStatus.OK, image.read_bytes(), "image/jpeg")
                return
            self._serve_static(parsed.path)
        except (OSError, RobotClientError) as exc:
            self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "error": str(exc)})
        except ValueError as exc:
            self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
        except Exception as exc:  # noqa: BLE001
            self.state.log(f"GET {parsed.path} 出错: {type(exc).__name__}: {exc}", "error")
            self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": f"{type(exc).__name__}: {exc}"})

    def _stream_mjpeg(self, path: Path) -> None:
        """把内存里的最新 JPEG 连续推给浏览器。断线就结束，不写磁盘。"""
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        last_ns = -1
        while True:
            try:
                stat = path.stat()
            except OSError:
                time.sleep(0.05)
                continue
            if stat.st_mtime_ns == last_ns or stat.st_size < 32:
                time.sleep(0.008)
                continue
            try:
                data = path.read_bytes()
            except OSError:
                time.sleep(0.008)
                continue
            if not data.startswith(b"\xff\xd8"):
                time.sleep(0.008)
                continue
            last_ns = stat.st_mtime_ns
            packet = (
                b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                + str(len(data)).encode("ascii")
                + b"\r\n\r\n"
                + data
                + b"\r\n"
            )
            try:
                self.wfile.write(packet)
                self.wfile.flush()
            except OSError:
                return

    def _serve_static(self, url_path: str) -> None:
        relative = ("index.html" if url_path in {"", "/"} else
                    "calibration.html" if url_path in {"/calibration", "/calibration/"} else url_path.lstrip("/"))
        candidate = (STATIC_ROOT / relative).resolve()
        try:
            candidate.relative_to(STATIC_ROOT.resolve())
        except ValueError:
            self._json(HTTPStatus.FORBIDDEN, {"ok": False, "error": "非法路径"})
            return
        if not candidate.is_file():
            self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "资源不存在"})
            return
        content_type = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
        if content_type.startswith("text/") or content_type in {"application/javascript", "application/json"}:
            content_type += "; charset=utf-8"
        self._send(HTTPStatus.OK, candidate.read_bytes(), content_type)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if not self._require_auth(parsed):
            return
        try:
            payload = self._parse_json()
            if parsed.path == '/api/maintenance-workcells':
                if not self.state.begin_action('切换维护机床组'):
                    self._json(HTTPStatus.CONFLICT, {'ok': False, 'error': '请等待当前动作完成再切换机床组'})
                    return
                action_error = ''
                try:
                    result = select_maintenance_workcell(self.state, payload.get('group'))
                    self.state.log('维护机床组: '+json.dumps(result, ensure_ascii=False), 'config')
                except Exception as error:
                    action_error = str(error)
                    raise
                finally:
                    self.state.end_action(action_error)
                self._json(HTTPStatus.OK, result)
                return
            if parsed.path == "/api/profiles":
                if not self.state.begin_action("保存机器人档案"):
                    self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "请等待当前网页动作完成"})
                    return
                action_error = ""
                try:
                    try:
                        ping = self.state.robot_request({"cmd": "ping"}, timeout=3.0)
                    except RobotClientError:
                        ping = None
                    if ping is not None:
                        if not ping.get("ok") or ping.get("mode") != "maintenance":
                            raise ValueError("请退出生产服务，使用维护模式修改下次启动档案")
                        calibration = self.state.robot_request({"cmd": "calibration_status"}, timeout=3.0)
                        if not calibration.get("ok") or calibration.get("active"):
                            raise ValueError("请先结束标定会话")
                    result = PROFILE_STORE.update(payload)
                    self.state.log("机器人档案: " + json.dumps(result, ensure_ascii=False), "profile")
                except Exception as exc:
                    action_error = str(exc)
                    raise
                finally:
                    self.state.end_action(action_error)
                self._json(HTTPStatus.OK, result)
                return
            if parsed.path == "/api/calibration":
                allowed = {"reset_stop", "begin", "prepare", "detect", "move_d", "jog", "observe",
                           "record_aligned", "release", "capture", "reference", "save", "return_ready", "end"}
                action = str(payload.get("action", ""))
                if action not in allowed:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "未知标定步骤"})
                    return
                if not self.state.begin_action("标定: " + action):
                    self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "已有动作在执行，请等待"})
                    return
                action_error = ""
                try:
                    payload["cmd"] = "placement_calibration"
                    result = command_result("placement_calibration", self.state.robot_request(payload))
                    if not result["ok"]:
                        action_error = result["error"]
                except Exception as exc:
                    action_error = str(exc)
                    raise
                finally:
                    self.state.end_action(action_error)
                self.state.log("标定 " + action + ": " + json.dumps(result, ensure_ascii=False), "calibration")
                self._json(HTTPStatus.OK, result)
                return
            if parsed.path in {"/api/fields", "/api/config"}:
                try: calibration = self.state.robot_request({"cmd": "calibration_status"}, timeout=3.0)
                except RobotClientError: calibration = {}
                if calibration.get("active"):
                    self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "标定期间禁止修改正式配置"})
                    return
            if parsed.path == "/api/command":
                cmd = str(payload.get("cmd", ""))
                if cmd not in ALLOWED_COMMANDS:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "命令不在允许列表"})
                    return
                if cmd in DANGEROUS_COMMANDS and payload.pop("confirm", "") != cmd:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "危险动作缺少二次确认"})
                    return
                if cmd not in {"abort", "dispatch_exit", "reload_config"} and not self.state.begin_action(cmd):
                    self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "已有动作在执行"})
                    return
                action_error = ""
                try:
                    # 调度模式一直跑到退出/停止，不能被 30 分钟超时断开后又放开网页的动作锁。
                    result = apply_persistent(self.state) if cmd == "reload_config" else command_result(cmd, self.state.robot_request(
                        payload, timeout=None if cmd == "dispatch" else 1800.0))
                    if not result["ok"]:
                        action_error = result["error"]
                except Exception as exc:
                    action_error = str(exc)
                    raise
                finally:
                    if cmd not in {"abort", "dispatch_exit", "reload_config"}:
                        self.state.end_action(action_error)
                self.state.log(f"命令 {cmd}: {json.dumps(result, ensure_ascii=False)}", "command")
                self._json(HTTPStatus.OK, result)
                return
            if parsed.path == "/api/sequence":
                if payload.get("confirm") != "sequence":
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "手动编排缺少二次确认"})
                    return
                steps = payload.get("steps")
                if not isinstance(steps, list) or not steps or any(step not in MANUAL_TASKS for step in steps):
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "编排只能包含上料、转运、下料"})
                    return
                if self.state.job_view().get("running"):
                    self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "已有动作在执行"})
                    return
                if not self.state.begin_action("手动编排"):
                    self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "已有动作在执行"})
                    return
                threading.Thread(target=self.state.run_sequence, args=(list(steps),), daemon=True).start()
                self._json(HTTPStatus.OK, {"ok": True, "steps": steps})
                return
            if parsed.path == "/api/fields":
                file_id = str(payload.get("file") or "motion")
                values = payload.get("values")
                if file_id not in FIELD_GROUPS:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "没有这份参数"})
                    return
                if payload.get("confirm") != "save_config":
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "保存配置缺少二次确认"})
                    return
                if not isinstance(values, dict) or not values:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "没有要保存的数"})
                    return
                try:
                    result = apply_persistent(self.state, file_id, render=lambda text: render_field_groups(
                        file_id, {str(k): str(v) for k, v in values.items()}, text))
                except KeyError as exc:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": f"找不到参数 {exc}"})
                    return
                self.state.log(result["message"], "config")
                self._json(HTTPStatus.OK, result)
                return
            if parsed.path == "/api/config":
                file_id = str(payload.get("file") or "motion")
                path = CONFIG_FILES.get(file_id)
                content = payload.get("content")
                if path is None:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "没有这份配置"})
                    return
                if payload.get("confirm") != "save_config":
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "保存配置缺少二次确认"})
                    return
                if not isinstance(content, str) or not content.strip() or len(content.encode("utf-8")) > 256 * 1024:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "配置内容为空或过大"})
                    return
                result = apply_persistent(self.state, file_id, content)
                self.state.log(result["message"], "config")
                self._json(HTTPStatus.OK, result)
                return
            if parsed.path == "/api/service/start":
                ok = self.state.start_robot()
                self._json(HTTPStatus.OK, {"ok": ok})
                return
            self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "API 不存在"})
        except ValueError as exc:
            self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
        except RobotClientError as exc:
            self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "error": str(exc)})
        except OSError as exc:
            self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": str(exc)})
        except Exception as exc:  # noqa: BLE001
            self.state.log(f"POST {parsed.path} 出错: {type(exc).__name__}: {exc}", "error")
            self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": f"{type(exc).__name__}: {exc}"})


class DebugHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], state: AppState) -> None:
        super().__init__(address, DebugHandler)
        self.app_state = state


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="T170C 机器人主机 Web 调试台")
    parser.add_argument("--host", default="0.0.0.0", help="Web 监听地址")
    parser.add_argument("--port", type=int, default=8080, help="Web 监听端口")
    parser.add_argument("--robot-host", default="127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=8099)
    parser.add_argument("--robot-command", default="./build_robot/t170_factory --maintenance")
    parser.add_argument("--start-robot", action="store_true", help="若 C++ 服务未运行则自动启动")
    parser.add_argument("--token", default=os.environ.get("T170_WEB_TOKEN", ""))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    state = AppState(args)
    if args.start_robot:
        try:
            state.start_robot()
        except RobotClientError as exc:
            state.log(str(exc), "error")
            print(f"[web] 警告：{exc}", file=sys.stderr)

    server = DebugHTTPServer((args.host, args.port), state)
    print("\nT170C Web 调试台已启动")
    print(f"  本机: http://127.0.0.1:{args.port}/?token={state.token}")
    print(f"  局域网: http://<机器人IP>:{args.port}/?token={state.token}")
    print("  访问令牌固定保存；重启沿用。首次通过上面的链接进入后，可直接收藏不带 token 的网址。")
    print("  实体急停必须保持可用；网页 STOP 不能替代实体急停。\n")

    def shutdown(_signum: int, _frame: Any) -> None:
        if state.shutdown_event.is_set():
            return
        state.shutdown_event.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGPIPE, signal.SIG_IGN)
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        state.stop_managed_robot()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
