#!/usr/bin/env python3
"""T170C robot-hosted debug web server.

The browser talks only to this process. Hardware remains exclusively owned by
the localhost C++ daemon (t170c_debug on 127.0.0.1:8099).
"""

from __future__ import annotations

import argparse
import hmac
import json
import mimetypes
import os
import secrets
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


PROJECT_ROOT = Path(__file__).resolve().parents[1]
STATIC_ROOT = Path(__file__).resolve().parent / "static"
CONFIG_PATH = PROJECT_ROOT / "config" / "move_box_params.yaml"
PICTURE_ROOT = PROJECT_ROOT / "picture_debug"
RUNTIME_ROOT = Path(__file__).resolve().parent / "runtime"
CAMERA_SLOTS = ("head", "right_hand", "left_hand")
ALLOWED_COMMANDS = {
    "ping",
    "snapshot",
    "reload_config",
    "vision_detect",
    "aruco_detect",
    "vision_grasp",
    "grasp_belt",
    "grasp_belt1",
    "dispatch",
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
    "tray2_place",
    "tray2_precision",
    "tray2test",
    "waist1",
    "waist2",
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
    "abort",
    "gripper_open",
    "gripper_grasp",
}
DANGEROUS_COMMANDS = {"vision_grasp", "grasp_belt", "grasp_belt1", "dispatch", "home", "grasp_ready", "grasp_ready1", "grasp_ready2", "grasp_ready3", "grasp_ready6", "tray2ready", "tray2ready1", "tray2ready2", "tray2ready3", "tray2ready6", "tray2", "tray2_place", "tray2_precision", "tray2test", "waist1", "waist2", "belt", "belt_ready", "belt_place", "belt_grasp_rpy", "belt_grasp", "belt2", "belt2_ready", "belt2_place", "belt2_grasp_rpy", "belt2_grasp", "waist_jog", "gripper_open", "gripper_grasp"}


class RobotClientError(RuntimeError):
    pass


class AppState:
    def __init__(self, args: argparse.Namespace) -> None:
        self.robot_host = args.robot_host
        self.robot_port = args.robot_port
        self.robot_command = args.robot_command
        self.token = args.token or secrets.token_urlsafe(18)
        self.token_generated = not bool(args.token)
        self.robot_process: subprocess.Popen[str] | None = None
        self.log_lines: deque[dict[str, Any]] = deque(maxlen=3000)
        self.log_seq = 0
        self.log_lock = threading.Lock()
        self.shutdown_event = threading.Event()
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

    def robot_request(self, payload: dict[str, Any], timeout: float = 1800.0) -> dict[str, Any]:
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

    def robot_online(self) -> bool:
        try:
            return bool(self.robot_request({"cmd": "ping"}, timeout=3.0).get("ok"))
        except RobotClientError:
            return False

    def start_robot(self) -> bool:
        if self.robot_online():
            self.log("检测到现有 t170c_debug，Web 调试台将连接该进程")
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
        "config_mtime_ms": int(CONFIG_PATH.stat().st_mtime * 1000) if CONFIG_PATH.is_file() else 0,
    }


class DebugHandler(BaseHTTPRequestHandler):
    server_version = "T170DebugWeb/1.0"

    @property
    def state(self) -> AppState:
        return self.server.app_state  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        # 浏览器首次 URL 和旧图片链接可能带 token；日志中必须脱敏。
        message = fmt % args
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
            if parsed.path == "/api/health":
                online = self.state.robot_online()
                process = self.state.robot_process
                self._json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        "robot_online": online,
                        "managed_process": process is not None,
                        "managed_process_running": process is not None and process.poll() is None,
                        "project_root": str(PROJECT_ROOT),
                        "host": host_status(),
                    },
                )
                return
            if parsed.path == "/api/status":
                self._json(HTTPStatus.OK, self.state.robot_request({"cmd": "snapshot"}, timeout=15.0))
                return
            if parsed.path == "/api/config":
                self._json(
                    HTTPStatus.OK,
                    {"ok": True, "path": str(CONFIG_PATH), "content": CONFIG_PATH.read_text(encoding="utf-8")},
                )
                return
            if parsed.path == "/api/logs":
                query = parse_qs(parsed.query)
                try:
                    since = int(query.get("since", ["0"])[0])
                except ValueError:
                    since = 0
                seq, lines = self.state.logs_since(max(0, since))
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

    def _serve_static(self, url_path: str) -> None:
        relative = "index.html" if url_path in {"", "/"} else url_path.lstrip("/")
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
        self._send(HTTPStatus.OK, candidate.read_bytes(), content_type, cache=candidate.name != "index.html")

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if not self._require_auth(parsed):
            return
        try:
            payload = self._parse_json()
            if parsed.path == "/api/command":
                cmd = str(payload.get("cmd", ""))
                if cmd not in ALLOWED_COMMANDS:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "命令不在允许列表"})
                    return
                if cmd in DANGEROUS_COMMANDS and payload.pop("confirm", "") != cmd:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "危险动作缺少二次确认"})
                    return
                result = self.state.robot_request(payload)
                self.state.log(f"命令 {cmd}: {json.dumps(result, ensure_ascii=False)}", "command")
                self._json(HTTPStatus.OK, result)
                return
            if parsed.path == "/api/config":
                content = payload.get("content")
                if payload.get("confirm") != "save_config":
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "保存配置缺少二次确认"})
                    return
                if not isinstance(content, str) or not content.strip() or len(content.encode("utf-8")) > 256 * 1024:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "配置内容为空或过大"})
                    return
                backup_dir = CONFIG_PATH.parent / ".web_backups"
                backup_dir.mkdir(parents=True, exist_ok=True)
                old_content = CONFIG_PATH.read_text(encoding="utf-8")
                backup = backup_dir / f"move_box_params_{datetime.now():%Y%m%d_%H%M%S}.yaml"
                backup.write_text(old_content, encoding="utf-8")
                temp = CONFIG_PATH.with_suffix(".yaml.web.tmp")
                temp.write_text(content, encoding="utf-8")
                os.replace(temp, CONFIG_PATH)
                try:
                    result = self.state.robot_request({"cmd": "reload_config"}, timeout=20.0)
                except RobotClientError:
                    CONFIG_PATH.write_text(old_content, encoding="utf-8")
                    raise
                if not result.get("ok"):
                    CONFIG_PATH.write_text(old_content, encoding="utf-8")
                    try:
                        self.state.robot_request({"cmd": "reload_config"}, timeout=20.0)
                    except RobotClientError:
                        pass
                    self._json(
                        HTTPStatus.BAD_REQUEST,
                        {"ok": False, "error": result.get("error", "配置校验失败"), "restored": True},
                    )
                    return
                self.state.log(f"配置已保存并重载，备份={backup.name}", "config")
                self._json(HTTPStatus.OK, {"ok": True, "backup": backup.name})
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
    parser.add_argument("--robot-command", default="./build_robot/t170c_debug")
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
    if state.token_generated:
        print("  访问令牌为本次随机生成；重启后会变化。可用 T170_WEB_TOKEN 固定。")
    print("  实体急停必须保持可用；网页 STOP 不能替代实体急停。\n")

    def shutdown(_signum: int, _frame: Any) -> None:
        if state.shutdown_event.is_set():
            return
        state.shutdown_event.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
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
