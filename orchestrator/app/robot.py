"""机器人后端接口。

当前默认 SimRobot：只打日志 + 按距离/时间等待，不碰硬件。
硬件对接点在 orchestrator/cpp/robot_runtime.h，由后续 C++ 守护进程实现。

底盘：不提供任何 chassis_* 方法。后期换底盘时在 RobotBackend 外另开 ChassisBackend。
"""

from __future__ import annotations

import json
import math
import subprocess
import threading
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Callable

from . import approach_paths as approach_paths_mod

LogFn = Callable[[str, str, str], None]  # level, source, message


def _norm_arm(arm: str) -> str:
    arm = (arm or "auto").lower()
    if arm in ("left", "l"):
        return "left"
    if arm in ("right", "r"):
        return "right"
    return "auto"


def _hover_path_fields(path: str) -> dict[str, Any]:
    name = (path or "").strip()
    if not name:
        return {}
    return approach_paths_mod.rpc_fields(name)


def _dist_pose(a: dict[str, float] | None, b: dict[str, float] | None) -> float:
    if not a or not b:
        return 0.3
    return math.sqrt((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2 + (a["z"] - b["z"]) ** 2)


class Abort(Exception):
    """紧急停止。"""


class RobotBackend(ABC):
    def __init__(self, log: LogFn, abort_event: threading.Event):
        self.log = log
        self.abort_event = abort_event
        self._tcp = {
            "left": {"x": 0.45, "y": 0.36, "z": -0.15, "rx": 90.0, "ry": 0.0, "rz": 0.0},
            "right": {"x": 0.45, "y": -0.36, "z": -0.15, "rx": -90.0, "ry": 0.0, "rz": 0.0},
        }
        self._gripper = {"left": "open", "right": "open"}
        self._holding = {"left": False, "right": False}
        self._last_placed = None
        self._last_conveyor = None
        self._waist = {"yaw_deg": 0.0, "pitch_deg": 0.0, "z_m": 0.0, "x_m": 0.0}
        self._head = [0.0, 40.0, 0.0]

    def snapshot(self) -> dict[str, Any]:
        return {
            "mode": self.mode_name(),
            "tcp": {k: dict(v) for k, v in self._tcp.items()},
            "gripper": dict(self._gripper),
            "waist": dict(self._waist),
            "head": list(self._head),
            "chassis": "disabled",
        }

    @abstractmethod
    def mode_name(self) -> str:
        ...

    def check_abort(self) -> None:
        if self.abort_event.is_set():
            raise Abort("紧急停止")

    def sleep_interruptible(self, seconds: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while True:
            self.check_abort()
            remain = end - time.monotonic()
            if remain <= 0:
                return
            time.sleep(min(0.05, remain))

    @abstractmethod
    def move_arm(self, side: str, pose: dict[str, float], speed: float) -> None:
        ...

    @abstractmethod
    def gripper(self, side: str, cmd: str) -> None:
        ...

    @abstractmethod
    def waist_rotate(self, angle_deg: float, speed_deg_s: float) -> None:
        ...

    @abstractmethod
    def waist_fold(self, angle_deg: float, speed_deg_s: float) -> None:
        ...

    @abstractmethod
    def waist_lift(self, delta_m: float, speed: float) -> None:
        ...

    @abstractmethod
    def waist_advance(self, delta_m: float, speed: float) -> None:
        ...

    @abstractmethod
    def waist_goto(self, pose: dict[str, float], speed: float) -> None:
        ...

    def hw_snapshot(self) -> dict[str, Any]:
        return self.snapshot()

    def reload_grasp_config(self) -> None:
        return

    def path_record(self, arm: str = "right", marker_id: int = 0, marker_id_b: int = 0) -> dict[str, Any]:
        raise RuntimeError("当前后端不支持路径示教")

    def modbus_wait(
        self, signal: int, timeout_sec: float = 120.0, host: str = "", addr: int = 0
    ) -> None:
        from .modbus_machine import wait_signal

        self.check_abort()
        wait_signal(
            int(signal),
            float(timeout_sec),
            log=self.log,
            check_abort=self.check_abort,
            sleep_s=self.sleep_interruptible,
            host=host,
            addr=addr or None,
        )

    def modbus_send(self, value: int, host: str = "", addr: int = 0) -> None:
        from .modbus_machine import send_value

        self.check_abort()
        send_value(int(value), log=self.log, host=host, addr=addr or None)

    @abstractmethod
    def home(self) -> None:
        ...

    @abstractmethod
    def vision_grasp(self) -> None:
        ...

    @abstractmethod
    def vision_detect(self) -> None:
        ...

    @abstractmethod
    def vision_grasp_factory(self) -> None:
        ...

    @abstractmethod
    def vision_detect_jindi(self) -> None:
        ...

    @abstractmethod
    def vision_grasp_jindi(self) -> None:
        ...

    @abstractmethod
    def aruco_detect(self) -> None:
        ...

    @abstractmethod
    def conveyor_goto(
        self, side: str, hand: str = "holding", arm: str = "auto", path: str = "", extras=None
    ) -> None:
        ...

    @abstractmethod
    def aruco_above(
        self, hand: str = "holding", arm: str = "auto", path: str = "", extras=None
    ) -> None:
        ...

    @abstractmethod
    def hand_grasp(self) -> None:
        ...

    @abstractmethod
    def hand_grasp_factory(self) -> None:
        ...

    @abstractmethod
    def vision_place(self) -> None:
        ...

    @abstractmethod
    def set_head(self, m0: float, m1: float, m2: float) -> None:
        ...


class SimRobot(RobotBackend):
    """无硬件仿真。耗时按笛卡尔距离 / 速度估算，便于把 UI 跑通。"""

    def mode_name(self) -> str:
        return "sim"

    def _travel(self, meters: float, speed: float, minimum: float = 0.25) -> float:
        speed = max(abs(speed), 1e-3)
        return max(minimum, abs(meters) / speed)

    def move_arm(self, side: str, pose: dict[str, float], speed: float) -> None:
        side = "left" if side == "left" else "right"
        label = "左臂" if side == "left" else "右臂"
        prev = self._tcp[side]
        dist = _dist_pose(prev, pose)
        self.log(
            "INFO",
            "arm",
            f"{label} 直线 {prev['x']:.3f},{prev['y']:.3f},{prev['z']:.3f} → "
            f"{pose['x']:.3f},{pose['y']:.3f},{pose['z']:.3f}  "
            f"rpy=({pose['rx']:.1f},{pose['ry']:.1f},{pose['rz']:.1f})°  v={speed:.2f}m/s",
        )
        # 映射现有 C++: arm_line_move / arm_dual_line_move_selective
        self.sleep_interruptible(self._travel(dist, speed))
        self._tcp[side] = dict(pose)
        self.log("INFO", "arm", f"{label} 到位")

    def gripper(self, side: str, cmd: str) -> None:
        side = "left" if side == "left" else "right"
        label = "左爪" if side == "left" else "右爪"
        verb = "松开" if cmd == "open" else "握住"
        self.log("INFO", "grip", f"{label} {verb}")
        # 映射现有 C++: gripper::Gripper openAndWait / graspAndWait
        self.sleep_interruptible(0.4)
        self._gripper[side] = cmd
        if cmd == "open":
            self._holding[side] = False
        self.log("INFO", "grip", f"{label} 完成")

    def waist_rotate(self, angle_deg: float, speed_deg_s: float) -> None:
        self.log("INFO", "waist", f"转腰 → {angle_deg:.1f}°  ({speed_deg_s:.0f}°/s)")
        # 映射: smooth_motor_move_deg(waist_id, yaw)
        dt = abs(angle_deg - self._waist["yaw_deg"]) / max(speed_deg_s, 1.0)
        self.sleep_interruptible(max(0.3, dt))
        self._waist["yaw_deg"] = angle_deg
        self.log("INFO", "waist", "转腰到位")

    def waist_fold(self, angle_deg: float, speed_deg_s: float) -> None:
        self.log("INFO", "waist", f"腰部折叠(pitch) → {angle_deg:.1f}°")
        dt = abs(angle_deg - self._waist["pitch_deg"]) / max(speed_deg_s, 1.0)
        self.sleep_interruptible(max(0.3, dt))
        self._waist["pitch_deg"] = angle_deg
        self.log("INFO", "waist", "折叠到位")

    def waist_lift(self, delta_m: float, speed: float) -> None:
        self.log("INFO", "waist", f"升降 Δz={delta_m:+.3f} m")
        # 映射: WaistRobot::moveLToPos z
        self.sleep_interruptible(self._travel(delta_m, speed, 0.3))
        self._waist["z_m"] += delta_m
        self.log("INFO", "waist", f"升降后 z≈{self._waist['z_m']:.3f} m")

    def waist_advance(self, delta_m: float, speed: float) -> None:
        self.log("INFO", "waist", f"前进 Δx={delta_m:+.3f} m（腰 stagger，非底盘）")
        self.sleep_interruptible(self._travel(delta_m, speed, 0.3))
        self._waist["x_m"] += delta_m
        self.log("INFO", "waist", f"前进后 x≈{self._waist['x_m']:.3f} m")

    def waist_goto(self, pose: dict[str, float], speed: float) -> None:
        self.check_abort()
        self.log(
            "INFO",
            "waist",
            f"腰到坐标 x={pose['x']:.3f} y={pose['y']:.3f} z={pose['z']:.3f} "
            f"rx={pose['rx']:.1f} ry={pose['ry']:.1f} rz={pose['rz']:.1f}",
        )
        self.sleep_interruptible(0.4)
        self._waist["x_m"] = pose["x"]
        self._waist["z_m"] = pose["z"]
        self.log("INFO", "waist", "腰坐标到位（仿真）")

    def hw_snapshot(self) -> dict[str, Any]:
        snap = self.snapshot()
        snap["waist_tcp"] = {
            "x": self._waist["x_m"],
            "y": 0.0,
            "z": self._waist["z_m"],
            "rx": -90.0,
            "ry": -90.0,
            "rz": 180.0,
        }
        return snap

    def home(self) -> None:
        self.log("INFO", "home", "双手回初始：腰 layer3_home + 双臂待机")
        self.sleep_interruptible(0.8)
        self._tcp["left"] = {"x": 0.45, "y": 0.36, "z": -0.15, "rx": 90.0, "ry": 0.0, "rz": 0.0}
        self._tcp["right"] = {"x": 0.45, "y": -0.36, "z": -0.15, "rx": -90.0, "ry": 0.0, "rz": 0.0}
        self.log("INFO", "home", "已到待机位")

    def vision_grasp(self) -> None:
        # 映射 main3 抓取主路径；算法本身不在此重写
        stages = [
            "开双爪",
            "头相机识别分配 (class0)",
            "手相机精定位",
            "接近并抓取",
            "抬起",
            "双臂回待命",
        ]
        self.log("INFO", "vision", "进入视觉抓取（仿真：打印现有 C++ 流程，不跑算法）")
        for i, name in enumerate(stages, 1):
            self.log("INFO", "vision", f"[{i}/{len(stages)}] {name}")
            self.sleep_interruptible(0.55)
        self._gripper["left"] = "close"
        self._gripper["right"] = "close"
        self._holding["left"] = True
        self._holding["right"] = True
        self.log("INFO", "vision", "视觉抓取仿真结束")

    def vision_detect(self) -> None:
        stages = [
            "头相机（工厂单类模型）",
            "右手相机",
            "左手相机",
        ]
        self.log("INFO", "vision", "进入视觉检测（仿真：不跑新 best.pt）")
        for i, name in enumerate(stages, 1):
            self.log("INFO", "vision", f"[{i}/{len(stages)}] {name}")
            self.sleep_interruptible(0.45)
        self.log("INFO", "vision", "视觉检测仿真结束 头=1 右=0 左=0")

    def vision_grasp_factory(self) -> None:
        stages = [
            "开双爪",
            "头相机识别分配（工厂单类）",
            "手相机精定位",
            "接近并抓取",
            "抬起",
            "双臂回待命",
        ]
        self.log("INFO", "vision", "进入视觉抓取(新模型)（仿真：不跑 best.pt）")
        for i, name in enumerate(stages, 1):
            self.log("INFO", "vision", f"[{i}/{len(stages)}] {name}")
            self.sleep_interruptible(0.55)
        self._gripper["left"] = "close"
        self._gripper["right"] = "close"
        self._holding["left"] = True
        self._holding["right"] = True
        self.log("INFO", "vision", "视觉抓取(新模型)仿真结束")

    def vision_detect_jindi(self) -> None:
        stages = ["头相机（金帝四类）", "右手相机", "左手相机"]
        self.log("INFO", "vision", "进入视觉检测(金帝四类)（仿真：不跑 seg_model/best.pt）")
        for i, name in enumerate(stages, 1):
            self.log("INFO", "vision", f"[{i}/{len(stages)}] {name}")
            self.sleep_interruptible(0.45)
        self.log("INFO", "vision", "金帝检测仿真结束 毛胚=1 半加工=0 加工=0 料盘孔=0")

    def vision_grasp_jindi(self) -> None:
        stages = [
            "开双爪",
            "头相机识别分配（金帝·毛胚）",
            "手相机精定位",
            "接近并抓取",
            "抬起",
            "双臂回待命",
        ]
        self.log("INFO", "vision", "进入视觉抓取(金帝·毛胚)（仿真：不跑 seg_model/best.pt）")
        for i, name in enumerate(stages, 1):
            self.log("INFO", "vision", f"[{i}/{len(stages)}] {name}")
            self.sleep_interruptible(0.55)
        self._gripper["left"] = "close"
        self._gripper["right"] = "close"
        self._holding["left"] = True
        self._holding["right"] = True
        self.log("INFO", "vision", "视觉抓取(金帝·毛胚)仿真结束")

    def aruco_detect(self) -> None:
        stages = ["头相机 ArUco", "右手相机 ArUco", "左手相机 ArUco"]
        self.log("INFO", "vision", "进入二维码位姿（仿真：不跑 ArUco）")
        for i, name in enumerate(stages, 1):
            self.log("INFO", "vision", f"[{i}/{len(stages)}] {name}")
            self.sleep_interruptible(0.45)
        self.log("INFO", "vision", "二维码位姿仿真结束 头=1 右=0 左=0")

    def conveyor_goto(
        self, side: str, hand: str = "holding", arm: str = "auto", path: str = "", extras=None
    ) -> None:
        side = "left" if side == "left" else "right"
        arm = _norm_arm(arm)
        want_holding = hand != "empty"
        extra = extras if extras is not None else _hover_path_fields(path)
        nwp = len(extra.get("waypoints") or [])
        if arm == "right":
            move_r = True
        elif arm == "left":
            move_r = False
        else:
            move_r = self._holding["right"] if want_holding else (not self._holding["right"])
            move_l = self._holding["left"] if want_holding else (not self._holding["left"])
            if not want_holding:
                if self._last_placed == "right":
                    move_r = False
                if self._last_placed == "left":
                    move_l = False
            if int(move_r) + int(move_l) != 1:
                move_r = side != "left"
                self.log("INFO", "conveyor", f"持料无法唯一选臂，仿真改走{'右' if move_r else '左'}臂")
        label = "左侧" if side == "left" else "右侧"
        pick = "指定臂" if arm in ("left", "right") else ("持料手" if want_holding else "空手")
        arm_cn = "右臂" if move_r else "左臂"
        path_t = f" 路径={path}({nwp}点)" if path else ""
        self.log("INFO", "conveyor", f"传送带{label}上方（仿真：{pick}→{arm_cn}{path_t}）")
        self.sleep_interruptible(0.4 + 0.25 * max(1, nwp))
        self._last_conveyor = "right" if move_r else "left"
        if arm == "auto" and want_holding:
            self._last_placed = self._last_conveyor
        self.log("INFO", "conveyor", f"传送带{label}上方仿真到位 {arm_cn}")

    def aruco_above(self, hand: str = "holding", arm: str = "auto", path: str = "", extras=None) -> None:
        arm = _norm_arm(arm)
        want_holding = hand != "empty"
        if arm == "right":
            move_r = True
        elif arm == "left":
            move_r = False
        else:
            move_r = self._holding["right"] if want_holding else (not self._holding["right"])
            move_l = self._holding["left"] if want_holding else (not self._holding["left"])
            if not want_holding:
                if self._last_placed == "right":
                    move_r = False
                if self._last_placed == "left":
                    move_l = False
            if int(move_r) + int(move_l) != 1:
                move_r = True
                self.log("INFO", "conveyor", "持料无法唯一选臂，仿真码正上方改走右臂")
        pick = "指定臂" if arm in ("left", "right") else ("持料手" if want_holding else "空手")
        arm_cn = "右臂" if move_r else "左臂"
        extra = extras if extras is not None else _hover_path_fields(path)
        nwp = len(extra.get("waypoints") or [])
        path_t = f" 路径={path}({nwp}点)" if path else ""
        self.log("INFO", "conveyor", f"二维码正上方约 5cm（仿真：{pick}→{arm_cn}{path_t}）")
        self.sleep_interruptible(0.4 + 0.25 * max(1, nwp))
        self._last_conveyor = "right" if move_r else "left"
        self.log("INFO", "conveyor", f"二维码上方仿真到位 {arm_cn}")

    def path_record(self, arm: str = "right", marker_id: int = 0, marker_id_b: int = 0) -> dict[str, Any]:
        arm = "left" if arm == "left" else "right"
        self.log("INFO", "path", f"仿真示教 {arm} 臂（假相对位姿）")
        tcp = dict(self._tcp[arm])
        return {
            "ok": True,
            "code": 0,
            "marker_id": int(marker_id or 113),
            "marker_id_b": int(marker_id_b or 0),
            "have_b": bool(marker_id_b),
            "tcp": tcp,
            "rel": {"x": -0.12, "y": 0.0, "z": 0.15, "rx": tcp["rx"], "ry": tcp["ry"], "rz": tcp["rz"]},
            "marker": {"x": 0.62, "y": -0.07, "z": -0.15, "rx": 0.0, "ry": 0.0, "rz": -88.0},
            "dual_rel": {"x": 0.08, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0} if marker_id_b else None,
            "markers": [
                {"slot": "head", "id": int(marker_id or 113), "robot_x": 0.62, "robot_y": -0.07, "robot_z": -0.15}
            ],
        }

    def hand_grasp(self) -> None:
        arm = self._last_conveyor
        stages = ["开爪", "手相机识别", "下压夹取", "抬起"]
        self.log("INFO", "vision", f"进入手部相机识别和抓取（仿真，{'只动' + ('右' if arm == 'right' else '左') if arm else '双手'}）")
        for i, name in enumerate(stages, 1):
            self.log("INFO", "vision", f"[{i}/{len(stages)}] {name}")
            self.sleep_interruptible(0.45)
        if arm in ("left", "right"):
            self._gripper[arm] = "close"
            self._holding[arm] = True
        else:
            self._gripper["left"] = "close"
            self._gripper["right"] = "close"
            self._holding["left"] = True
            self._holding["right"] = True
        self.log("INFO", "vision", "手部相机抓取仿真结束")

    def hand_grasp_factory(self) -> None:
        self.hand_grasp()
        self.log("INFO", "vision", "手部相机抓取(小毛胚)仿真结束")

    def vision_place(self) -> None:
        stages = [
            "头相机空位识别(class1)",
            "持料手到空位上方",
            "下压",
            "松开持料手",
            "回升待命",
        ]
        holding = [s for s, v in self._holding.items() if v]
        self.log("INFO", "vision", f"进入视觉放货（仿真：持料={holding or '无'}）")
        if not holding:
            raise RuntimeError("没有持料手，请先抓取")
        for i, name in enumerate(stages, 1):
            self.log("INFO", "vision", f"[{i}/{len(stages)}] {name}")
            self.sleep_interruptible(0.5)
        for side in holding:
            self._gripper[side] = "open"
            self._holding[side] = False
        self.log("INFO", "vision", "视觉放货仿真结束")

    def set_head(self, m0: float, m1: float, m2: float) -> None:
        self.log("INFO", "head", f"头部电机 → [{m0:.1f}, {m1:.1f}, {m2:.1f}]°")
        # 映射: socketcan_sendcommand(head_id, can 30/31/32)
        self.sleep_interruptible(0.25)
        self._head = [m0, m1, m2]


class HwRobot(RobotBackend):
    """通过 127.0.0.1:8099 调 orch_hw。底盘命令直接拒绝。"""

    def __init__(self, log: LogFn, abort_event: threading.Event, host: str = "127.0.0.1", port: int = 8099):
        super().__init__(log, abort_event)
        self.host = host
        self.port = port
        self._proc = None
        self._stderr_thread = None
        self._ensure_daemon()
        threading.Thread(target=self._abort_watch, name="hw-abort", daemon=True).start()

    def mode_name(self) -> str:
        return "hw"

    def _bin_path(self) -> str:
        root = Path(__file__).resolve().parents[2]
        for p in (root / "build" / "orch_hw", root / "orch_hw"):
            if p.is_file():
                return str(p)
        return str(root / "build" / "orch_hw")

    def _ensure_daemon(self) -> None:
        try:
            self._rpc_once({"cmd": "ping"}, timeout=2)
            self.log("INFO", "hw", f"已连接 orch_hw {self.host}:{self.port}")
            return
        except OSError:
            pass

        bin_path = self._bin_path()
        if not Path(bin_path).is_file():
            raise RuntimeError(
                f"找不到 {bin_path}。请先 cmake --build build --target orch_hw"
            )
        self.log("INFO", "hw", f"启动 {bin_path}")
        self._proc = subprocess.Popen(
            [bin_path, "--port", str(self.port)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=str(Path(bin_path).parent.parent),
        )
        self._stderr_thread = threading.Thread(target=self._pump_stdout, daemon=True)
        self._stderr_thread.start()
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            try:
                self._rpc_once({"cmd": "ping"}, timeout=1)
                self.log("INFO", "hw", "orch_hw 就绪（无底盘）")
                return
            except OSError:
                if self._proc.poll() is not None:
                    raise RuntimeError("orch_hw 启动失败，见终端日志")
                time.sleep(0.4)
        raise RuntimeError("等待 orch_hw 超时")

    def _pump_stdout(self) -> None:
        if not self._proc or not self._proc.stdout:
            return
        for line in self._proc.stdout:
            line = line.rstrip()
            if line:
                self.log("INFO", "hw", line)

    def _abort_watch(self) -> None:
        while True:
            self.abort_event.wait()
            while self.abort_event.is_set():
                try:
                    self._rpc_once({"cmd": "abort"}, timeout=2)
                except Exception:
                    pass
                time.sleep(0.2)

    def _rpc_once(self, payload: dict, timeout: float = 300) -> dict:
        import socket as _socket

        data = (json.dumps(payload, ensure_ascii=False) + "\n").encode()
        with _socket.create_connection((self.host, self.port), timeout=timeout) as s:
            s.sendall(data)
            buf = b""
            while b"\n" not in buf:
                chunk = s.recv(4096)
                if not chunk:
                    break
                buf += chunk
        if not buf:
            raise RuntimeError("orch_hw 无响应")
        reply = json.loads(buf.decode())
        if not reply.get("ok", False):
            err = reply.get("error") or "硬件命令失败"
            if err == "已中止" or "紧急停止" in err:
                raise Abort(err)
            raise RuntimeError(err)
        return reply

    def _rpc(self, payload: dict, timeout: float = 300) -> dict:
        try:
            return self._rpc_once(payload, timeout)
        except OSError as exc:
            if payload.get("cmd") in ("ping", "abort"):
                raise
            self.log("WARN", "hw", f"orch_hw 断开，尝试重启: {exc}")
            self._ensure_daemon()
            return self._rpc_once(payload, timeout)
        except RuntimeError as exc:
            if str(exc) != "orch_hw 无响应" or payload.get("cmd") in ("ping", "abort"):
                raise
            self.log("WARN", "hw", f"orch_hw 断开，尝试重启: {exc}")
            self._ensure_daemon()
            return self._rpc_once(payload, timeout)

    def move_arm(self, side: str, pose: dict[str, float], speed: float) -> None:
        self.check_abort()
        self.log("INFO", "arm", f"{'左臂' if side == 'left' else '右臂'} 下发硬件直线")
        self._rpc({"cmd": "arm", "side": side, "pose": pose, "speed": speed})
        self._tcp[side] = dict(pose)

    def gripper(self, side: str, cmd: str) -> None:
        self.check_abort()
        self._rpc({"cmd": "gripper", "side": side, "action": cmd})
        self._gripper[side] = cmd
        if cmd == "open":
            self._holding[side] = False

    def waist_rotate(self, angle_deg: float, speed_deg_s: float) -> None:
        self.check_abort()
        self._rpc({"cmd": "waist_rotate", "angle_deg": angle_deg, "speed_deg_s": speed_deg_s})
        self._waist["yaw_deg"] = angle_deg

    def waist_fold(self, angle_deg: float, speed_deg_s: float) -> None:
        self.check_abort()
        self._rpc({"cmd": "waist_fold", "angle_deg": angle_deg, "speed_deg_s": speed_deg_s})
        self._waist["pitch_deg"] = angle_deg

    def waist_lift(self, delta_m: float, speed: float) -> None:
        self.check_abort()
        self._rpc({"cmd": "waist_delta", "dx": 0.0, "dz": delta_m, "speed": speed})
        self._waist["z_m"] += delta_m

    def waist_advance(self, delta_m: float, speed: float) -> None:
        self.check_abort()
        self._rpc({"cmd": "waist_delta", "dx": delta_m, "dz": 0.0, "speed": speed})
        self._waist["x_m"] += delta_m

    def waist_goto(self, pose: dict[str, float], speed: float) -> None:
        self.check_abort()
        self.log(
            "INFO",
            "waist",
            f"腰到坐标 x={pose['x']:.3f} y={pose['y']:.3f} z={pose['z']:.3f}",
        )
        self._rpc({"cmd": "waist_goto", "pose": pose, "speed": speed}, timeout=120)
        self._waist["x_m"] = pose["x"]
        self._waist["z_m"] = pose["z"]

    def hw_snapshot(self) -> dict[str, Any]:
        try:
            reply = self._rpc_once({"cmd": "snapshot"}, timeout=3)
        except Exception:
            return self.snapshot()
        if reply.get("tcp"):
            self._tcp = {k: dict(v) for k, v in reply["tcp"].items()}
        wt = reply.get("waist_tcp")
        if wt:
            self._waist["x_m"] = float(wt.get("x") or 0)
            self._waist["z_m"] = float(wt.get("z") or 0)
        snap = self.snapshot()
        snap["waist_tcp"] = wt
        snap["tcp"] = reply.get("tcp") or snap["tcp"]
        if reply.get("links"):
            snap["links"] = reply["links"]
        return snap

    def reload_grasp_config(self) -> None:
        try:
            self._rpc_once({"cmd": "reload_config"}, timeout=5)
            self.log("INFO", "cfg", "orch_hw 已重载 move_box_params.yaml")
        except OSError:
            self.log("WARN", "cfg", "orch_hw 未连接，参数已写入 yaml，下次启动生效")

    def home(self) -> None:
        self.check_abort()
        self.log("INFO", "home", "硬件归位：先抬手回待机，再腰 layer3_home")
        self._rpc({"cmd": "home"}, timeout=120)

    def vision_grasp(self) -> None:
        self.check_abort()
        self.log("INFO", "vision", "调用 C++ 视觉抓取：识别→上方→下压→抬起→待命")
        reply = self._rpc({"cmd": "vision_grasp"}, timeout=600)
        self.log(
            "INFO",
            "vision",
            f"抓取结束 code={reply.get('code')} 右={reply.get('grasped_r')} 左={reply.get('grasped_l')}",
        )
        if reply.get("grasped_r"):
            self._gripper["right"] = "close"
        if reply.get("grasped_l"):
            self._gripper["left"] = "close"

    def vision_detect(self) -> None:
        self.check_abort()
        self.log("INFO", "vision", "调用 C++ 工厂单类检测（根目录 best.pt，不抓取）")
        reply = self._rpc({"cmd": "vision_detect"}, timeout=180)
        self.log(
            "INFO",
            "vision",
            f"检测结束 code={reply.get('code')} 头={reply.get('head')} "
            f"右={reply.get('right')} 左={reply.get('left')}",
        )

    def vision_grasp_factory(self) -> None:
        self.check_abort()
        self.log("INFO", "vision", "调用 C++ 工厂单类抓取（根目录 best.pt）")
        reply = self._rpc({"cmd": "vision_grasp_factory"}, timeout=600)
        self.log(
            "INFO",
            "vision",
            f"抓取结束 code={reply.get('code')} 右={reply.get('grasped_r')} 左={reply.get('grasped_l')}",
        )
        if reply.get("grasped_r"):
            self._gripper["right"] = "close"
        if reply.get("grasped_l"):
            self._gripper["left"] = "close"

    def vision_detect_jindi(self) -> None:
        self.check_abort()
        self.log("INFO", "vision", "调用 C++ 金帝四类检测（seg_model/best.pt，不抓取）")
        reply = self._rpc({"cmd": "vision_detect_jindi"}, timeout=180)
        detail = reply.get("detail") or (
            f"毛胚={reply.get('blank', 0)} 半加工={reply.get('semi', 0)} "
            f"加工={reply.get('finished', 0)} 料盘孔={reply.get('hole', 0)}"
        )
        self.log(
            "INFO",
            "vision",
            f"金帝检测结束 code={reply.get('code')} 头={reply.get('head')} "
            f"右={reply.get('right')} 左={reply.get('left')} {detail}",
        )

    def vision_grasp_jindi(self) -> None:
        self.check_abort()
        self.log("INFO", "vision", "调用 C++ 金帝四类抓取（只抓毛胚 class0）")
        reply = self._rpc({"cmd": "vision_grasp_jindi"}, timeout=600)
        self.log(
            "INFO",
            "vision",
            f"抓取结束 code={reply.get('code')} 右={reply.get('grasped_r')} 左={reply.get('grasped_l')}",
        )
        if reply.get("grasped_r"):
            self._gripper["right"] = "close"
        if reply.get("grasped_l"):
            self._gripper["left"] = "close"

    def aruco_detect(self) -> None:
        self.check_abort()
        self.log("INFO", "vision", "调用 C++ ArUco 单码位姿（按码 ID 查边长）")
        reply = self._rpc({"cmd": "aruco_detect"}, timeout=180)
        self.log(
            "INFO",
            "vision",
            f"ArUco 结束 code={reply.get('code')} 头={reply.get('head')} "
            f"右={reply.get('right')} 左={reply.get('left')}",
        )
        for m in reply.get("markers") or []:
            slot = m.get("slot", "?")
            mid = m.get("id")
            side = m.get("side_mm")
            cam = f"cam=({m.get('cam_x'):.3f},{m.get('cam_y'):.3f},{m.get('cam_z'):.3f})"
            extra = ""
            if "robot_x" in m:
                extra = (
                    f" robot=({m.get('robot_x'):.3f},{m.get('robot_y'):.3f},"
                    f"{m.get('robot_z'):.3f})"
                )
            self.log("INFO", "vision", f"  [{slot}] ID={mid} L={side:.0f}mm {cam}{extra}")

    def conveyor_goto(
        self, side: str, hand: str = "holding", arm: str = "auto", path: str = "", extras=None
    ) -> None:
        self.check_abort()
        side = "left" if side == "left" else "right"
        hand = "empty" if hand == "empty" else "holding"
        arm = _norm_arm(arm)
        extra = extras if extras is not None else _hover_path_fields(path)
        nwp = len(extra.get("waypoints") or [])
        label = "左侧" if side == "left" else "右侧"
        arm_cn = {"left": "左臂", "right": "右臂"}.get(arm, "自动选臂")
        path_t = f" 路径={path}({nwp}点)" if path else ""
        self.log("INFO", "conveyor", f"调用 C++ 传送带{label}上方（{arm_cn}{path_t}）")
        payload = {"cmd": "conveyor_goto", "side": side, "hand": hand, "arm": arm, **extra}
        reply = self._rpc(payload, timeout=180)
        used = reply.get("arm") or "?"
        dual = ""
        if float(reply.get("dual_pos_m") or -1) >= 0:
            dual = (
                f" 双码误差 {float(reply['dual_pos_m']):.4f}m/"
                f"{float(reply.get('dual_rot_deg') or 0):.2f}°"
                f"{' 已修正' if reply.get('dual_corrected') else ' 未修正'}"
            )
        self.log(
            "INFO",
            "conveyor",
            f"传送带{label}上方结束 code={reply.get('code')} 臂={used} "
            f"主码={reply.get('marker_id')} 头码={reply.get('head')}{dual}",
        )

    def aruco_above(self, hand: str = "holding", arm: str = "auto", path: str = "", extras=None) -> None:
        self.check_abort()
        hand = "empty" if hand == "empty" else "holding"
        arm = _norm_arm(arm)
        extra = extras if extras is not None else _hover_path_fields(path)
        nwp = len(extra.get("waypoints") or [])
        arm_cn = {"left": "左臂", "right": "右臂"}.get(arm, "自动选臂")
        path_t = f" 路径={path}({nwp}点)" if path else ""
        self.log("INFO", "conveyor", f"调用 C++ 二维码正上方约 5cm（{arm_cn}{path_t}）")
        reply = self._rpc({"cmd": "aruco_above", "hand": hand, "arm": arm, **extra}, timeout=180)
        used = reply.get("arm") or "?"
        self.log(
            "INFO",
            "conveyor",
            f"二维码上方结束 code={reply.get('code')} 臂={used} "
            f"持料右={reply.get('holding_r')} 左={reply.get('holding_l')} 头码={reply.get('head')}",
        )

    def path_record(self, arm: str = "right", marker_id: int = 0, marker_id_b: int = 0) -> dict[str, Any]:
        self.check_abort()
        arm = "left" if arm == "left" else "right"
        self.log("INFO", "path", f"示教记录当前 TCP（{arm}）相对头相机二维码")
        reply = self._rpc(
            {
                "cmd": "path_record",
                "arm": arm,
                "marker_id": int(marker_id or 0),
                "marker_id_b": int(marker_id_b or 0),
            },
            timeout=60,
        )
        rel = reply.get("rel") or {}
        self.log(
            "INFO",
            "path",
            f"已记录 主码={reply.get('marker_id')} 副码={reply.get('marker_id_b')} "
            f"相对 xyz=({rel.get('x', 0):.4f},{rel.get('y', 0):.4f},{rel.get('z', 0):.4f})",
        )
        return reply

    def hand_grasp(self) -> None:
        self.check_abort()
        self.log("INFO", "vision", "调用 C++ 手相机识别+抓取（不回待命）")
        reply = self._rpc({"cmd": "hand_grasp"}, timeout=600)
        self.log(
            "INFO",
            "vision",
            f"手相机抓取结束 code={reply.get('code')} 右={reply.get('grasped_r')} 左={reply.get('grasped_l')}",
        )
        if reply.get("grasped_r"):
            self._gripper["right"] = "close"
        if reply.get("grasped_l"):
            self._gripper["left"] = "close"

    def hand_grasp_factory(self) -> None:
        self.check_abort()
        self.log("INFO", "vision", "调用 C++ 手相机识别+抓取(小毛胚)")
        reply = self._rpc({"cmd": "hand_grasp_factory"}, timeout=600)
        self.log(
            "INFO",
            "vision",
            f"手相机抓取(小毛胚)结束 code={reply.get('code')} 右={reply.get('grasped_r')} 左={reply.get('grasped_l')}",
        )
        if reply.get("grasped_r"):
            self._gripper["right"] = "close"
        if reply.get("grasped_l"):
            self._gripper["left"] = "close"

    def vision_place(self) -> None:
        self.check_abort()
        self.log("INFO", "vision", "调用 C++ 视觉放货（4类 class1 空位）")
        reply = self._rpc({"cmd": "vision_place"}, timeout=300)
        self.log(
            "INFO",
            "vision",
            f"放货结束 code={reply.get('code')} 空位={reply.get('holes')} "
            f"松右={reply.get('released_r')} 松左={reply.get('released_l')} "
            f"持料右={reply.get('holding_r')} 左={reply.get('holding_l')}",
        )
        if reply.get("released_r"):
            self._gripper["right"] = "open"
            self._holding["right"] = False
        if reply.get("released_l"):
            self._gripper["left"] = "open"
            self._holding["left"] = False

    def set_head(self, m0: float, m1: float, m2: float) -> None:
        self.check_abort()
        self._rpc({"cmd": "head", "m0": m0, "m1": m1, "m2": m2})
        self._head = [m0, m1, m2]


def create_robot(mode: str, log: LogFn, abort_event: threading.Event) -> RobotBackend:
    mode = (mode or "hw").lower()
    if mode in ("hw", "hardware"):
        log("INFO", "robot", "硬件模式：将连接 orch_hw（底盘禁用）")
        return HwRobot(log, abort_event)
    return SimRobot(log, abort_event)
