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
import re
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

import yaml


# 前端 app.js 里有同一个数，对不上时页面提示重启 server.py 并强制刷新。
WEB_API_VERSION = 4
UI_VERSION = "1.1.6"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
STATIC_ROOT = Path(__file__).resolve().parent / "static"
CONFIG_FILES = {
    "motion": PROJECT_ROOT / "config" / "move_box_params.yaml",
    "tray_code": PROJECT_ROOT / "board_yf100_aruco" / "config.yaml",
    "belt_code": PROJECT_ROOT / "board_belt_aruco" / "config.yaml",
    "hand_eye": PROJECT_ROOT / "free_calib" / "config" / "camera_to_head_connector_result.yaml",
    "head_geom": PROJECT_ROOT / "free_calib" / "config" / "head_params.yaml",
    "hand_eye_fallback": PROJECT_ROOT / "config" / "camera_to_base_result.yaml",
}
CONFIG_NOTES = {
    "motion": "保存后会立刻 reload。手臂模型、新代码仍要重启 t170c_debug。",
    "tray_code": "料盘二维码。保存后要重启 t170c_debug 才生效。",
    "belt_code": "传送带二维码。保存后要重启 t170c_debug 才生效。",
    "hand_eye": "相机到头部的平移。保存后要重启 t170c_debug。程序读矩阵，平移会同时写进矩阵。",
    "head_geom": "头部离基座的高度。保存后要重启 t170c_debug。",
    "hand_eye_fallback": "手眼失败时的回退。这一页仍是原文，保存后要重启。",
}
MANUAL_TASKS = {
    "load": ["vision_grasp", "belt"],
    "transfer": ["belt_grasp", "belt2"],
    "unload": ["belt2_grasp", "tray2"],
}
MANUAL_LABELS = {"load": "上料", "transfer": "转运", "unload": "下料"}

# 只暴露平时会改的数。path 用点号，列表用下标。保存时按行替换，注释保留。
FIELD_GROUPS = {
    "motion": [
        ("腰", [
            ("waist.ready_forward_m", "料盘抓取时，腰相对 home 再向前", "米"),
            ("waist.far_row_forward_m", "料盘一远三排，腰再向前伸", "米"),
            ("waist.ready_z_m", "抓取时腰的高度", "米"),
            ("waist.layer3_home.2", "回到 home 时腰的高度", "米"),
            ("waist.move_settle_sec", "腰到位后再停一会儿", "秒"),
            ("tray2_place.far_row_forward_m", "料盘二放置时，远排腰向前伸", "米"),
        ]),
        ("头", [
            ("head.pitch_deg", "平时低头角度", "度"),
            ("head.far_pitch_deg", "看远排时再低头的角度", "度"),
            ("head.speed_deg_s", "头转动速度", "度/秒"),
        ]),
        ("料盘高度", [
            ("tray.z_ref_m", "料盘上表面在手臂基座下的高度", "米"),
            ("head_grasp.hover_above_m", "手停在孔上方的高度", "米"),
            ("head_grasp.lift_after_grasp_z", "夹住后再抬起的高度", "米"),
            ("head_grasp.pre_grasp_settle_sec", "下到孔里后再等一会儿才合爪", "秒"),
            ("head_grasp.bezier_endpoint_xyz_tol_m", "手离目标超过这个距离就不合爪", "米"),
        ]),
        ("料盘抓取速度", [
            ("head_grasp.approach_vel_m_s", "伸向孔位的速度", "米/秒"),
            ("head_grasp.descend_vel_m_s", "下压速度", "米/秒"),
            ("head_grasp.lift_vel_m_s", "抬起速度", "米/秒"),
            ("head_grasp.return_vel_m_s", "收回速度", "米/秒"),
        ]),
        ("料盘抓取偏置，左手", [
            ("head_grasp.goal_x_offset", "前后，向前为正", "米"),
            ("head_grasp.goal_y_offset", "左右，向左为正", "米"),
            ("head_grasp.grasp_z_offset_m", "上下，向上为正", "米"),
        ]),
        ("料盘抓取偏置，右手", [
            ("head_grasp.right_goal_x_offset", "前后，向前为正", "米"),
            ("head_grasp.right_goal_y_offset", "左右，向左为正", "米"),
            ("head_grasp.right_grasp_z_offset_m", "上下，向上为正", "米"),
        ]),
        ("料盘最远一排的手腕角度，左手", [
            ("head_grasp.row1_rpy_deg.left.0", "翻滚", "度"),
            ("head_grasp.row1_rpy_deg.left.1", "俯仰", "度"),
            ("head_grasp.row1_rpy_deg.left.2", "偏航", "度"),
        ]),
        ("料盘最远一排的手腕角度，右手", [
            ("head_grasp.row1_rpy_deg.right.0", "翻滚", "度"),
            ("head_grasp.row1_rpy_deg.right.1", "俯仰", "度"),
            ("head_grasp.row1_rpy_deg.right.2", "偏航", "度"),
        ]),
        ("料盘二放置偏置，左手", [
            ("tray2_place.offset.left.x", "前后，向前为正", "米"),
            ("tray2_place.offset.left.y", "左右，向左为正", "米"),
            ("tray2_place.offset.left.z", "上下，向上为正", "米"),
        ]),
        ("料盘二放置偏置，右手", [
            ("tray2_place.offset.right.x", "前后，向前为正", "米"),
            ("tray2_place.offset.right.y", "左右，向左为正", "米"),
            ("tray2_place.offset.right.z", "上下，向上为正", "米"),
        ]),
        ("传送带一", [
            ("conveyor.waist_x", "拍照时腰向前", "米"),
            ("conveyor.waist_z", "拍照时腰的高度", "米"),
            ("conveyor.head_pitch_deg", "头低头角度", "度"),
            ("conveyor.grasp_hover_above_m", "抓取时先停在上方", "米"),
            ("conveyor.place_hover_above_m", "放置时先停在上方", "米"),
            ("conveyor.grasp_ready_close_ratio", "下去前夹爪合上的程度，0 全开 1 全合", ""),
            ("conveyor.fixed_origin_z_m", "码平面在基座下的固定高度", "米"),
            ("conveyor.photo_after_arrival_sec", "到站后放置拍照前再等", "秒"),
            ("conveyor.grasp_photo_settle_sec", "腰头到位后抓取拍照前再等", "秒"),
            ("conveyor.offset.right.x", "右手放置前后", "米"),
            ("conveyor.offset.right.y", "右手放置左右", "米"),
            ("conveyor.offset.right.z", "右手放置上下", "米"),
            ("conveyor.offset.left.x", "左手放置前后", "米"),
            ("conveyor.offset.left.y", "左手放置左右", "米"),
            ("conveyor.offset.left.z", "左手放置上下", "米"),
            ("conveyor.grasp_offset.right.x", "右手抓取前后", "米"),
            ("conveyor.grasp_offset.right.y", "右手抓取左右", "米"),
            ("conveyor.grasp_offset.right.z", "右手抓取上下", "米"),
            ("conveyor.grasp_offset.left.x", "左手抓取前后", "米"),
            ("conveyor.grasp_offset.left.y", "左手抓取左右", "米"),
            ("conveyor.grasp_offset.left.z", "左手抓取上下", "米"),
        ]),
        ("传送带二", [
            ("conveyor2.waist_x", "拍照时腰向前", "米"),
            ("conveyor2.waist_z", "拍照时腰的高度", "米"),
            ("conveyor2.head_pitch_deg", "头低头角度", "度"),
            ("conveyor2.grasp_hover_above_m", "抓取时先停在上方", "米"),
            ("conveyor2.place_hover_above_m", "放置时先停在上方", "米"),
            ("conveyor2.fixed_origin_z_m", "码平面固定高度，要勾上使用才生效", "米"),
            ("conveyor2.photo_after_arrival_sec", "到站后放置拍照前再等", "秒"),
            ("conveyor2.grasp_photo_settle_sec", "腰头到位后抓取拍照前再等", "秒"),
            ("conveyor2.offset.right.x", "右手放置前后", "米"),
            ("conveyor2.offset.right.y", "右手放置左右", "米"),
            ("conveyor2.offset.right.z", "右手放置上下", "米"),
            ("conveyor2.offset.left.x", "左手放置前后", "米"),
            ("conveyor2.offset.left.y", "左手放置左右", "米"),
            ("conveyor2.offset.left.z", "左手放置上下", "米"),
            ("conveyor2.grasp_offset.right.x", "右手抓取前后", "米"),
            ("conveyor2.grasp_offset.right.y", "右手抓取左右", "米"),
            ("conveyor2.grasp_offset.right.z", "右手抓取上下", "米"),
            ("conveyor2.grasp_offset.left.x", "左手抓取前后", "米"),
            ("conveyor2.grasp_offset.left.y", "左手抓取左右", "米"),
            ("conveyor2.grasp_offset.left.z", "左手抓取上下", "米"),
        ]),
        ("空抓检测", [
            ("gripper_check.enable", "合爪后判空抓，true 开 false 关", ""),
            ("gripper_check.empty_q_rad", "合爪结束 q 不大于这个值算空抓，要明显小于夹住零件时的 q", "弧度"),
            ("gripper_check.max_empty_retry", "同一只手空抓后最多重抓几次", "次"),
        ]),
        ("站点", [
            ("chassis.tray_station", "料盘一", ""),
            ("chassis.belt_station", "传送带一", ""),
            ("chassis.out_station", "传送带二", ""),
            ("chassis.tray2_station", "料盘二", ""),
            ("chassis.dispatch_wait_station", "调度等待点（失败/无任务回这里）", ""),
            ("chassis.charge_station", "调试充电点（charge 指令；调度仍用系统下发点）", ""),
            ("chassis.charge_wait_station", "充完电后待命点", ""),
            ("chassis.charge_done_percent", "调度充电结束电量阈值", "%"),
        ]),
    ],
    "tray_code": [
        ("正在使用的四角码", [
            ("tray_grasp_code.pitch_xy_m", "四个码中心的间距", "米"),
            ("tray_grasp_code.print_size_m", "打印外边长", "米"),
            ("tray_grasp_code.black_side_m", "黑框边长", "米"),
            ("tray_grasp_code.locate_id", "左上和右下常见的码号", ""),
        ]),
        ("料盘二，和料盘一同一套四角码，不按号码分角", [
            ("tray2.print_size_m", "打印外边长", "米"),
            ("tray2.black_side_m", "黑框边长", "米"),
            ("tray2.pitch_xy_m", "四个码中心的间距", "米"),
        ]),
    ],
    "belt_code": [
        ("传送带码", [
            ("aruco.print_size_m", "打印外边长", "米"),
            ("belt.pitch_y_m", "两个码心的左右间距", "米"),
            ("belt.edge.front_m", "码心到皮带前端", "米"),
            ("belt.edge.rear_m", "码心到皮带后端", "米"),
            ("belt.edge.left_m", "码心到皮带左端", "米"),
            ("belt.edge.right_m", "码心到皮带右端", "米"),
        ]),
    ],
    "hand_eye": [
        ("相机到头部的平移，程序读的是矩阵，改这里会写回矩阵最后一列", [
            ("tx", "相机相对头部，前后方向的平移", "毫米"),
            ("ty", "相机相对头部，左右方向的平移", "毫米"),
            ("tz", "相机相对头部，上下方向的平移", "毫米"),
        ]),
    ],
    "head_geom": [
        ("头部高度", [
            ("link_length_mm", "头部电机末端离基座的高度", "毫米"),
        ]),
    ],
}


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


def _replace_yaml_value(text: str, path: str, value_text: str) -> str:
    parts = path.split(".")
    lines = text.splitlines()
    stack: list[tuple[int, str]] = []
    for index, line in enumerate(lines):
        stripped = line.lstrip(" ")
        if not stripped or stripped.startswith("#"):
            continue
        indent = len(line) - len(stripped)
        code, hash_sep, comment = stripped.partition("#")
        if ":" not in code:
            continue
        key, _, rest = code.partition(":")
        key = key.strip()
        rest = rest.strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        here = [item[1] for item in stack] + [key]
        comment_s = f"  #{comment}" if hash_sep else ""
        if here == parts:
            lines[index] = " " * indent + f"{key}: {value_text}" + comment_s
            break
        if (
            len(parts) == len(here) + 1
            and parts[: len(here)] == here
            and parts[-1].isdigit()
            and rest.startswith("[")
            and "]" in rest
        ):
            slot = int(parts[-1])
            body = rest[1:rest.index("]")]
            nums = [item.strip() for item in body.split(",")]
            if slot >= len(nums):
                raise KeyError(path)
            nums[slot] = value_text
            lines[index] = " " * indent + f"{key}: [" + ", ".join(nums) + "]" + comment_s
            break
        if (
            len(parts) == len(here) + 1
            and parts[: len(here)] == here
            and rest.startswith("{")
        ):
            child = parts[-1]
            new_rest, count = re.subn(
                rf"({re.escape(child)}\s*:\s*)[-+0-9.eE]+",
                rf"\g<1>{value_text}",
                rest,
                count=1,
            )
            if count != 1:
                raise KeyError(path)
            lines[index] = " " * indent + f"{key}: {new_rest}" + comment_s
            break
        if rest == "" or rest.startswith("|") or rest.startswith(">"):
            stack.append((indent, key))
    else:
        raise KeyError(path)
    ending = "\n" if text.endswith("\n") else ""
    return "\n".join(lines) + ending


def _sync_hand_eye_matrix(text: str, xyz: list[float]) -> str:
    """平移写在 b. translation 和 a. matrix 的最后一列。旋转不动。"""
    import ast

    match = re.search(r"(a\. matrix:\s*)(\[\[.*?\]\])", text, re.S)
    if not match:
        raise KeyError("a. matrix")
    matrix = ast.literal_eval(match.group(2))
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise KeyError("a. matrix")
    for row, value in enumerate(xyz):
        matrix[row][3] = value
    rendered = "[" + ", ".join("[" + ", ".join(repr(float(n)) for n in row) + "]" for row in matrix) + "]"
    return text[: match.start(2)] + rendered + text[match.end(2) :]


def read_field_groups(file_id: str) -> list[dict[str, Any]]:
    path = CONFIG_FILES[file_id]
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


def write_field_groups(file_id: str, updates: dict[str, str]) -> None:
    path = CONFIG_FILES[file_id]
    text = path.read_text(encoding="utf-8")
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
        rendered = "[" + ", ".join(format(v, ".8g") for v in vec) + "]"
        text, count = re.subn(
            r"(?m)^(b\. translation_xyz_mm:\s*)\[[^\]]*\]",
            rf"\g<1>{rendered}",
            text,
            count=1,
        )
        if count != 1:
            raise KeyError("b. translation_xyz_mm")
        text = _sync_hand_eye_matrix(text, vec)
    else:
        for key, raw in updates.items():
            text = _replace_yaml_value(text, key, str(raw).strip())
    backup_dir = path.parent / ".web_backups"
    backup_dir.mkdir(parents=True, exist_ok=True)
    backup = backup_dir / f"{path.name}_{datetime.now():%Y%m%d_%H%M%S}.bak"
    backup.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    temp = path.with_suffix(path.suffix + ".web.tmp")
    temp.write_text(text, encoding="utf-8")
    os.replace(temp, path)

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
    "vision_grasp_once",
    "grasp_belt",
    "grasp_belt1",
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
    "tray2_place",
    "tray2_precision",
    "tray2test",
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
    "cycle3",
    "tray2center",
    "detect_tray_holes",
    "abort",
    "dispatch_exit",
    "gripper_open",
    "gripper_grasp",
}
DANGEROUS_COMMANDS = ALLOWED_COMMANDS - {
    "ping",
    "snapshot",
    "reload_config",
    "vision_detect",
    "aruco_detect",
    "detect_tray_holes",
    "abort",
    "dispatch_exit",
}


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
                "title": "",
                "step": "已停止" if error else "空闲",
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
                    result = self.robot_request({"cmd": cmd})
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
                        "api": WEB_API_VERSION,
                        "ui_version": UI_VERSION,
                        "robot_online": online,
                        "managed_process": process is not None,
                        "managed_process_running": process is not None and process.poll() is None,
                        "project_root": str(PROJECT_ROOT),
                        "host": host_status(),
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
                    result["error"] = "t170c_debug 是旧版本，重启它之后才有调度面板"
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
                        "groups": read_field_groups(file_id),
                    },
                )
                return
            if parsed.path == "/api/config":
                file_id = parse_qs(parsed.query).get("file", ["motion"])[0]
                path = CONFIG_FILES.get(file_id)
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
        except Exception as exc:  # noqa: BLE001
            self.state.log(f"GET {parsed.path} 出错: {type(exc).__name__}: {exc}", "error")
            self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": f"{type(exc).__name__}: {exc}"})

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
        self._send(HTTPStatus.OK, candidate.read_bytes(), content_type)

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
                if cmd not in {"abort", "dispatch_exit"} and not self.state.begin_action(cmd):
                    self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "已有动作在执行"})
                    return
                try:
                    # 调度模式一直跑到退出/停止，不能被 30 分钟超时断开后又放开网页的动作锁。
                    result = self.state.robot_request(
                        payload, timeout=None if cmd == "dispatch" else 1800.0
                    )
                finally:
                    if cmd not in {"abort", "dispatch_exit"}:
                        self.state.end_action()
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
                    write_field_groups(file_id, {str(k): str(v) for k, v in values.items()})
                except KeyError as exc:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": f"找不到参数 {exc}"})
                    return
                except ValueError:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "有数字填得不对"})
                    return
                if file_id != "motion":
                    self.state.log(f"{file_id} 参数已保存，重启 t170c_debug 后生效", "config")
                    self._json(HTTPStatus.OK, {"ok": True, "reload": False})
                    return
                try:
                    result = self.state.robot_request({"cmd": "reload_config"}, timeout=20.0)
                except RobotClientError:
                    self._json(HTTPStatus.OK, {"ok": True, "reload": False, "error": "已写入文件，但机器人没在运行，未能 reload"})
                    return
                if not result.get("ok"):
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": result.get("error", "重载失败")})
                    return
                self.state.log("运动参数已保存并重载", "config")
                self._json(HTTPStatus.OK, {"ok": True, "reload": True})
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
                backup_dir = path.parent / ".web_backups"
                backup_dir.mkdir(parents=True, exist_ok=True)
                old_content = path.read_text(encoding="utf-8") if path.is_file() else ""
                backup = backup_dir / f"{path.name}_{datetime.now():%Y%m%d_%H%M%S}.bak"
                backup.write_text(old_content, encoding="utf-8")
                temp = path.with_suffix(path.suffix + ".web.tmp")
                temp.write_text(content, encoding="utf-8")
                os.replace(temp, path)
                if file_id != "motion":
                    self.state.log(f"{path.name} 已保存，重启 t170c_debug 后生效，备份={backup.name}", "config")
                    self._json(HTTPStatus.OK, {"ok": True, "backup": backup.name, "reload": False})
                    return
                try:
                    result = self.state.robot_request({"cmd": "reload_config"}, timeout=20.0)
                except RobotClientError:
                    path.write_text(old_content, encoding="utf-8")
                    raise
                if not result.get("ok"):
                    path.write_text(old_content, encoding="utf-8")
                    try:
                        self.state.robot_request({"cmd": "reload_config"}, timeout=20.0)
                    except RobotClientError:
                        pass
                    self._json(
                        HTTPStatus.BAD_REQUEST,
                        {"ok": False, "error": result.get("error", "配置校验失败"), "restored": True},
                    )
                    return
                self.state.log(f"运动参数已保存并重载，备份={backup.name}", "config")
                self._json(HTTPStatus.OK, {"ok": True, "backup": backup.name, "reload": True})
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
