"""序列执行器：按步顺序跑，双臂/双爪同一步双线程。"""

from __future__ import annotations

import threading
from concurrent.futures import ThreadPoolExecutor, wait
from typing import Any, Callable

from .robot import Abort, RobotBackend
from .schema import canonicalize_step, step_label, validate_program

StateFn = Callable[[dict[str, Any]], None]


class SequenceRunner:
    def __init__(self, robot: RobotBackend, log, on_state: StateFn):
        self.robot = robot
        self.log = log
        self.on_state = on_state
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self.status = "idle"  # idle | running | stopping | error
        self.current_id: str | None = None
        self.error: str | None = None
        self.loop_index = 0
        self.loop_total = 1

    def snapshot(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "current_id": self.current_id,
            "error": self.error,
            "loop_index": self.loop_index,
            "loop_total": self.loop_total,
            "robot": self.robot.snapshot(),
        }

    def _publish(self) -> None:
        self.on_state(self.snapshot())

    def is_busy(self) -> bool:
        return self.status in ("running", "stopping")

    def request_stop(self) -> None:
        if not self.is_busy():
            self.log("WARN", "run", "当前没有在跑的序列")
            return
        self.status = "stopping"
        self.robot.abort_event.set()
        self.log("WARN", "run", "紧急停止：通知所有动作线程退出")
        self._publish()

    def start(self, steps: list[dict[str, Any]], *, loops: int = 1, from_id: str | None = None) -> None:
        with self._lock:
            if self.is_busy():
                raise RuntimeError("序列正在运行")
            errors = validate_program(steps)
            if errors:
                raise ValueError("；".join(errors))
            clean = [canonicalize_step(s) for s in steps]
            if from_id:
                idx = next((i for i, s in enumerate(clean) if s["id"] == from_id), None)
                if idx is None:
                    raise ValueError("起始步骤不存在")
                clean = clean[idx:]
            self.robot.abort_event.clear()
            self.status = "running"
            self.error = None
            self.current_id = None
            self.loop_total = int(loops)
            if self.loop_total < 0:
                self.loop_total = 0  # 0 = 无限
            self.loop_index = 0
            self._thread = threading.Thread(
                target=self._run_safe, args=(clean,), name="sequence-runner", daemon=True
            )
            self._thread.start()
        self._publish()

    def _run_safe(self, steps: list[dict[str, Any]]) -> None:
        try:
            self._run(steps)
            if self.robot.abort_event.is_set():
                self.status = "idle"
                self.log("WARN", "run", "序列已中止")
            else:
                self.status = "idle"
                self.log("INFO", "run", "序列完成")
        except Abort:
            self.status = "idle"
            self.log("WARN", "run", "序列已中止")
        except Exception as exc:  # noqa: BLE001 — 推到前端日志
            self.status = "error"
            self.error = str(exc)
            self.log("ERROR", "run", f"序列失败: {exc}")
        finally:
            self.current_id = None
            self.robot.abort_event.clear()
            self._publish()

    def _run(self, steps: list[dict[str, Any]]) -> None:
        infinite = self.loop_total <= 0
        loop_i = 0
        while True:
            self.robot.check_abort()
            loop_i += 1
            self.loop_index = loop_i
            if infinite:
                self.log("INFO", "run", f"第 {self.loop_index} 圈（无限，STOP 可停）")
            elif self.loop_total > 1:
                self.log("INFO", "run", f"第 {self.loop_index}/{self.loop_total} 圈")
            for i, step in enumerate(steps, 1):
                self.robot.check_abort()
                self.current_id = step["id"]
                self._publish()
                title = step_label(step)
                note = f"  // {step['note']}" if step.get("note") else ""
                self.log("INFO", "run", f"-- 开始 [{i}/{len(steps)}] {title}{note}")
                self._exec_step(step)
                self.log("INFO", "run", f"-- 结束 [{i}/{len(steps)}] {title}")
            if not infinite and loop_i >= self.loop_total:
                break
        self.current_id = None
        self._publish()

    def _exec_step(self, step: dict[str, Any]) -> None:
        kind = step["kind"]
        if kind == "sleep":
            self.log("INFO", "sleep", f"等待 {step['seconds']:.2f}s")
            self.robot.sleep_interruptible(float(step["seconds"]))
            return
        if kind == "home":
            self.robot.home()
            return
        if kind == "vision_grasp":
            self.robot.vision_grasp()
            return
        if kind == "vision_detect":
            self.robot.vision_detect()
            return
        if kind == "vision_grasp_factory":
            self.robot.vision_grasp_factory()
            return
        if kind == "vision_detect_jindi":
            self.robot.vision_detect_jindi()
            return
        if kind == "vision_grasp_jindi":
            self.robot.vision_grasp_jindi()
            return
        if kind == "aruco_detect":
            self.robot.aruco_detect()
            return
        if kind == "conveyor_goto":
            self.robot.conveyor_goto(
                str(step.get("side") or "right"),
                str(step.get("hand") or "holding"),
                str(step.get("arm") or "auto"),
                str(step.get("path") or ""),
            )
            return
        if kind == "aruco_above":
            self.robot.aruco_above(
                str(step.get("hand") or "holding"),
                str(step.get("arm") or "auto"),
                str(step.get("path") or ""),
            )
            return
        if kind == "hand_grasp":
            self.robot.hand_grasp()
            return
        if kind == "hand_grasp_factory":
            self.robot.hand_grasp_factory()
            return
        if kind == "vision_place":
            self.robot.vision_place()
            return
        if kind == "head":
            self.robot.set_head(float(step["m0"]), float(step["m1"]), float(step["m2"]))
            return
        if kind == "modbus_wait":
            self.robot.modbus_wait(
                int(step["signal"]),
                float(step.get("timeout_sec") or 120),
                host=str(step.get("host") or ""),
                addr=int(step.get("addr") or 0),
            )
            return
        if kind == "modbus_send":
            self.robot.modbus_send(
                int(step["value"]),
                host=str(step.get("host") or ""),
                addr=int(step.get("addr") or 0),
            )
            return
        if kind == "waist":
            mode = step["mode"]
            if mode == "rotate":
                self.robot.waist_rotate(step["angle_deg"], step["speed_deg_s"])
            elif mode == "fold":
                self.robot.waist_fold(step["angle_deg"], step["speed_deg_s"])
            elif mode == "lift":
                self.robot.waist_lift(step["delta_m"], step["speed"])
            elif mode == "advance":
                self.robot.waist_advance(step["delta_m"], step["speed"])
            elif mode == "goto":
                self.robot.waist_goto(step["pose"], step["speed"])
            return
        if kind == "arm":
            jobs = []
            if step.get("left"):
                jobs.append(("left", step["left"], step["speed"]))
            if step.get("right"):
                jobs.append(("right", step["right"], step["speed"]))
            self._parallel(jobs, lambda side, pose, speed: self.robot.move_arm(side, pose, speed))
            return
        if kind == "gripper":
            jobs = []
            if step.get("left"):
                jobs.append(("left", step["left"]))
            if step.get("right"):
                jobs.append(("right", step["right"]))
            self._parallel(jobs, lambda side, cmd: self.robot.gripper(side, cmd))
            return
        raise ValueError(f"未实现的动作: {kind}")

    def _parallel(self, jobs: list[tuple], fn) -> None:
        if not jobs:
            return
        if len(jobs) == 1:
            fn(*jobs[0])
            return
        self.log("INFO", "run", f"双线程并发 {len(jobs)} 路")
        errors: list[BaseException] = []

        def wrap(job):
            try:
                fn(*job)
            except Abort:
                raise
            except Exception as exc:  # noqa: BLE001
                errors.append(exc)
                self.log("ERROR", "run", f"并发支路失败: {exc}")

        with ThreadPoolExecutor(max_workers=len(jobs), thread_name_prefix="dual") as pool:
            futs = [pool.submit(wrap, job) for job in jobs]
            wait(futs)
        # 把子线程 Abort 冒泡
        for fut in futs:
            exc = fut.exception()
            if isinstance(exc, Abort):
                raise exc
        if errors:
            raise RuntimeError(errors[0])
