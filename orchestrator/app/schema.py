"""动作 JSON schema 与合并/校验。

假设（与现有 C++ 对齐）：
- 手臂位姿: x/y/z 米，rx/ry/rz 欧拉角度（与 standby yaml 相同）
- 双臂/双爪在同一步内并发；序列按步顺序执行
- 底盘动作不进入本 schema，后期换底盘再加 chassis.* 命名空间
"""

from __future__ import annotations

import copy
import uuid
from typing import Any

SCHEMA_VERSION = 1

# 与 config/move_box_params.yaml standby 一致
DEFAULT_POSE_RIGHT = {"x": 0.45, "y": -0.36, "z": -0.15, "rx": -90.0, "ry": 0.0, "rz": 0.0}
DEFAULT_POSE_LEFT = {"x": 0.45, "y": 0.36, "z": -0.15, "rx": 90.0, "ry": 0.0, "rz": 0.0}

ARM_KINDS = {"arm"}
GRIPPER_KINDS = {"gripper"}
WAIST_MODES = {"rotate", "fold", "lift", "advance", "goto"}
GRIPPER_CMDS = {"open", "close"}

LIBRARY = [
    {
        "id": "arm_left",
        "kind": "arm",
        "title": "左臂",
        "group": "手臂",
        "hint": "笛卡尔直线。若当前步已有右臂且左臂空，则合并为双臂并发。",
        "seed": {"kind": "arm", "left": dict(DEFAULT_POSE_LEFT), "right": None, "speed": 0.2},
    },
    {
        "id": "arm_right",
        "kind": "arm",
        "title": "右臂",
        "group": "手臂",
        "hint": "笛卡尔直线。若当前步已有左臂且右臂空，则合并为双臂并发。",
        "seed": {"kind": "arm", "left": None, "right": dict(DEFAULT_POSE_RIGHT), "speed": 0.2},
    },
    {
        "id": "arm_dual",
        "kind": "arm",
        "title": "双臂",
        "group": "手臂",
        "hint": "左右同时直线，后端双线程。",
        "seed": {
            "kind": "arm",
            "left": dict(DEFAULT_POSE_LEFT),
            "right": dict(DEFAULT_POSE_RIGHT),
            "speed": 0.2,
        },
    },
    {
        "id": "grip_open",
        "kind": "gripper",
        "title": "夹爪松开",
        "group": "夹爪",
        "hint": "默认同开双爪；可在卡片里关掉一侧。",
        "seed": {"kind": "gripper", "left": "open", "right": "open"},
    },
    {
        "id": "grip_close",
        "kind": "gripper",
        "title": "夹爪握住",
        "group": "夹爪",
        "hint": "默认同握双爪；力矩保护由现有 DEX1 接口负责。",
        "seed": {"kind": "gripper", "left": "close", "right": "close"},
    },
    {
        "id": "waist_rotate",
        "kind": "waist",
        "title": "转腰",
        "group": "腰部",
        "hint": "腰 1 轴 yaw（度）。对应 smooth_motor_move_deg(waist)。",
        "seed": {"kind": "waist", "mode": "rotate", "angle_deg": 0.0, "speed_deg_s": 30.0},
    },
    {
        "id": "waist_fold",
        "kind": "waist",
        "title": "腰部折叠",
        "group": "腰部",
        "hint": "腰 2 轴 pitch（度）。第 6 排弯腰同思路。",
        "seed": {"kind": "waist", "mode": "fold", "angle_deg": 20.0, "speed_deg_s": 30.0},
    },
    {
        "id": "lift_up",
        "kind": "waist",
        "title": "上升",
        "group": "升降",
        "hint": "腰 TCP 沿 z 正向。底盘升降不在本工具范围。",
        "seed": {"kind": "waist", "mode": "lift", "delta_m": 0.05, "speed": 0.1},
    },
    {
        "id": "lift_down",
        "kind": "waist",
        "title": "下降",
        "group": "升降",
        "hint": "腰 TCP 沿 z 负向。",
        "seed": {"kind": "waist", "mode": "lift", "delta_m": -0.05, "speed": 0.1},
    },
    {
        "id": "advance",
        "kind": "waist",
        "title": "前进",
        "group": "升降",
        "hint": "腰 stagger 前进（米）。不是底盘走点。",
        "seed": {"kind": "waist", "mode": "advance", "delta_m": 0.06, "speed": 0.1},
    },
    {
        "id": "waist_goto",
        "kind": "waist",
        "title": "腰到坐标",
        "group": "腰部",
        "hint": "腰 TCP 走到指定 x/y/z/姿态。添加时自动填入当前腰位置，不是从 0 开始。",
        "seed": {
            "kind": "waist",
            "mode": "goto",
            "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "rx": -90.0, "ry": -90.0, "rz": 180.0},
            "speed": 0.1,
        },
    },
    {
        "id": "sleep",
        "kind": "sleep",
        "title": "等待",
        "group": "流程",
        "hint": "阻塞等待，便于手动观察。",
        "seed": {"kind": "sleep", "seconds": 1.0},
    },
    {
        "id": "home",
        "kind": "home",
        "title": "双手回初始",
        "group": "流程",
        "hint": "先竖直抬手到待机高度，再直线回 standby，最后腰 yaw=0 + layer3_home。避免手先停在料盘上方时转腰/斜插扫台。",
        "seed": {"kind": "home"},
    },
    {
        "id": "vision_grasp",
        "kind": "vision_grasp",
        "title": "视觉抓取",
        "group": "视觉",
        "hint": "头相机识别 → 到物体上方 → 手相机改 xy → 下压夹取 → 抬起 → 回待命。视觉算法不在前端实现。",
        "seed": {"kind": "vision_grasp"},
    },
    {
        "id": "vision_detect",
        "kind": "vision_detect",
        "title": "视觉检测",
        "group": "视觉",
        "hint": "用工程根目录单类 best.pt 拍三相机并出位姿，只识别不抓取。旧 4 类模型不动。",
        "seed": {"kind": "vision_detect"},
    },
    {
        "id": "vision_grasp_factory",
        "kind": "vision_grasp_factory",
        "title": "视觉抓取(新模型)",
        "group": "视觉",
        "hint": "用根目录单类 best.pt 走与「视觉抓取」相同流程：识别→物体上方→下压夹取→抬起→回待命。旧 4 类模型不动。",
        "seed": {"kind": "vision_grasp_factory"},
    },
    {
        "id": "vision_detect_jindi",
        "kind": "vision_detect_jindi",
        "title": "视觉检测(金帝四类)",
        "group": "视觉",
        "hint": "用 seg_model/best.pt 识别毛胚 / 半加工 / 加工 / 料盘孔，三相机只识别不抓取。",
        "seed": {"kind": "vision_detect_jindi"},
    },
    {
        "id": "vision_grasp_jindi",
        "kind": "vision_grasp_jindi",
        "title": "视觉抓取(金帝·毛胚)",
        "group": "视觉",
        "hint": "金帝四类模型，只抓 class0 毛胚：识别→物体上方→下压→抬起→回待命。半加工/加工/孔只识别不抓。",
        "seed": {"kind": "vision_grasp_jindi"},
    },
    {
        "id": "aruco_detect",
        "kind": "aruco_detect",
        "title": "二维码位姿",
        "group": "视觉",
        "hint": "头/左/右手相机检测 ArUco，按码 ID 查边长（online_pose/config.yaml）再结合内参解位姿。只识别不抓取。",
        "seed": {"kind": "aruco_detect"},
    },
    {
        "id": "conveyor_right",
        "kind": "conveyor_goto",
        "title": "传送带右侧上方",
        "group": "传送带",
        "hint": "头相机找码，指定手臂先走示教相对路径点（可选），再到码右侧上方。默认右臂。",
        "seed": {"kind": "conveyor_goto", "side": "right", "arm": "right", "hand": "holding", "path": ""},
    },
    {
        "id": "conveyor_left",
        "kind": "conveyor_goto",
        "title": "传送带左侧上方",
        "group": "传送带",
        "hint": "头相机找码，指定手臂先走示教相对路径点（可选），再到码左侧上方。挡板传送带取料用 left_pick。",
        "seed": {"kind": "conveyor_goto", "side": "left", "arm": "left", "hand": "empty", "path": "left_pick"},
    },
    {
        "id": "aruco_above",
        "kind": "aruco_above",
        "title": "二维码上方",
        "group": "传送带",
        "hint": "头相机找码，指定手臂先走示教相对路径点（可选），再到码正上方约 5cm。",
        "seed": {"kind": "aruco_above", "arm": "right", "hand": "holding", "path": ""},
    },
    {
        "id": "hand_grasp",
        "kind": "hand_grasp",
        "title": "手部相机识别和抓取",
        "group": "视觉",
        "hint": "识别 → 到物体上方 → 下压夹取 → 抬起。用上一动作到位的那只手，不回头相机、不回待命。接传送带后只动空手。旧 4 类模型。",
        "seed": {"kind": "hand_grasp"},
    },
    {
        "id": "hand_grasp_factory",
        "kind": "hand_grasp_factory",
        "title": "手部相机识别和抓取(小毛胚)",
        "group": "视觉",
        "hint": "同上（工厂单类 best.pt / 小毛胚）：识别 → 物体上方 → 下压 → 抬起。",
        "seed": {"kind": "hand_grasp_factory"},
    },
    {
        "id": "vision_place",
        "kind": "vision_place",
        "title": "视觉放货",
        "group": "视觉",
        "hint": "头相机用旧 4 类模型找 class1 空穴，持料手放回料盘。无底盘、无第6排弯腰。腰先转到料盘高度。",
        "seed": {"kind": "vision_place"},
    },
    {
        "id": "head",
        "kind": "head",
        "title": "头部姿态",
        "group": "头部",
        "hint": "头三电机角度（度）。启动默认 0° / 40° / 0°。",
        "seed": {"kind": "head", "m0": 0.0, "m1": 40.0, "m2": 0.0},
    },
    {
        "id": "modbus_wait_10",
        "kind": "modbus_wait",
        "title": "等待上料许可(10)",
        "group": "机床信号",
        "hint": "同时轮询 192.168.1.88 和 .89 的上料许可(10@2100)与下料许可(20@2200)。本步等到任意一台出现 10 后继续。",
        "seed": {"kind": "modbus_wait", "signal": 10, "timeout_sec": 120, "host": "", "addr": 0},
    },
    {
        "id": "modbus_wait_20",
        "kind": "modbus_wait",
        "title": "等待下料许可(20)",
        "group": "机床信号",
        "hint": "同时轮询 192.168.1.88 和 .89 的上料许可(10@2100)与下料许可(20@2200)。本步等到任意一台出现 20 后继续。",
        "seed": {"kind": "modbus_wait", "signal": 20, "timeout_sec": 120, "host": "", "addr": 0},
    },
    {
        "id": "modbus_send",
        "kind": "modbus_send",
        "title": "发送机床信号",
        "group": "机床信号",
        "hint": "写入保持寄存器。35/15→2150，45/25→2250。主机空=最近等到许可的那台；若还没等到则两台都写。",
        "seed": {"kind": "modbus_send", "value": 35, "host": "", "addr": 0},
    },
]


def new_id() -> str:
    return uuid.uuid4().hex[:10]


def normalize_pose(raw: Any) -> dict[str, float] | None:
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ValueError("位姿必须是对象 {x,y,z,rx,ry,rz}")
    out = {}
    for key in ("x", "y", "z", "rx", "ry", "rz"):
        if key not in raw or raw[key] is None or raw[key] == "":
            raise ValueError(f"位姿缺少 {key}")
        out[key] = float(raw[key])
    return out


def boot_standby_from_steps(steps: list[dict[str, Any]]) -> dict[str, Any]:
    """启动流程里最后一次双臂目标，作为抓取/归位用的待机位。"""
    poses: dict[str, Any] = {}
    speed = 0.2
    for step in steps:
        if step.get("kind") != "arm":
            continue
        if step.get("left"):
            poses["left"] = dict(step["left"])
        if step.get("right"):
            poses["right"] = dict(step["right"])
        if step.get("speed") is not None:
            speed = float(step["speed"])
    poses["speed"] = speed
    return poses


def boot_waist_from_steps(steps: list[dict[str, Any]]) -> dict[str, Any] | None:
    """启动流程里最后一次腰 goto，作为现用 layer3_home。"""
    pose = None
    for step in steps:
        if step.get("kind") == "waist" and step.get("mode") == "goto" and step.get("pose"):
            pose = dict(step["pose"])
    return pose


def make_step(kind: str, params: dict[str, Any] | None = None, *, note: str = "") -> dict[str, Any]:
    params = copy.deepcopy(params or {})
    step: dict[str, Any] = {
        "id": new_id(),
        "kind": kind,
        "note": note,
        **params,
    }
    if "kind" not in params:
        step["kind"] = kind
    return canonicalize_step(step)


def canonicalize_step(step: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(step, dict):
        raise ValueError("步骤必须是对象")
    kind = step.get("kind")
    if not kind:
        raise ValueError("步骤缺少 kind")
    out: dict[str, Any] = {
        "id": str(step.get("id") or new_id()),
        "kind": kind,
        "note": str(step.get("note") or ""),
    }
    if kind == "arm":
        out["left"] = normalize_pose(step.get("left"))
        out["right"] = normalize_pose(step.get("right"))
        out["speed"] = float(step.get("speed") if step.get("speed") not in (None, "") else 0.2)
        if out["speed"] <= 0:
            raise ValueError("手臂速度必须 > 0")
        if out["left"] is None and out["right"] is None:
            raise ValueError("手臂动作至少要有一侧目标")
    elif kind == "gripper":
        left = step.get("left")
        right = step.get("right")
        out["left"] = None if left in (None, "", "none") else str(left)
        out["right"] = None if right in (None, "", "none") else str(right)
        for side in ("left", "right"):
            if out[side] is not None and out[side] not in GRIPPER_CMDS:
                raise ValueError(f"夹爪{side} 只能是 open/close")
        if out["left"] is None and out["right"] is None:
            raise ValueError("夹爪动作至少要有一侧")
    elif kind == "waist":
        mode = str(step.get("mode") or "rotate")
        if mode not in WAIST_MODES:
            raise ValueError(f"未知腰部模式: {mode}")
        out["mode"] = mode
        out["angle_deg"] = float(step.get("angle_deg") or 0.0)
        out["delta_m"] = float(step.get("delta_m") or 0.0)
        out["speed"] = float(step.get("speed") or 0.1)
        out["speed_deg_s"] = float(step.get("speed_deg_s") or 30.0)
        if mode == "goto":
            pose = step.get("pose")
            if pose is None:
                pose = {k: step.get(k) for k in ("x", "y", "z", "rx", "ry", "rz")}
            out["pose"] = normalize_pose(pose)
    elif kind == "sleep":
        out["seconds"] = float(step.get("seconds") or 0.0)
        if out["seconds"] < 0:
            raise ValueError("等待时间不能为负")
    elif kind == "home":
        pass
    elif kind == "vision_grasp":
        pass
    elif kind == "vision_detect":
        pass
    elif kind == "vision_grasp_factory":
        pass
    elif kind == "vision_detect_jindi":
        pass
    elif kind == "vision_grasp_jindi":
        pass
    elif kind == "aruco_detect":
        pass
    elif kind == "conveyor_goto":
        side = str(step.get("side") or "right")
        if side not in ("left", "right"):
            raise ValueError("传送带定位 side 只能是 left/right")
        out["side"] = side
        arm = str(step.get("arm") or "auto")
        if arm not in ("auto", "left", "right"):
            raise ValueError("传送带 arm 只能是 auto/left/right")
        out["arm"] = arm
        hand = str(step.get("hand") or ("empty" if side == "left" else "holding"))
        if hand not in ("holding", "empty"):
            raise ValueError("传送带 hand 只能是 holding/empty")
        out["hand"] = hand
        out["path"] = str(step.get("path") or "").strip()
    elif kind == "aruco_above":
        arm = str(step.get("arm") or "auto")
        if arm not in ("auto", "left", "right"):
            raise ValueError("二维码上方 arm 只能是 auto/left/right")
        out["arm"] = arm
        hand = str(step.get("hand") or "holding")
        if hand not in ("holding", "empty"):
            raise ValueError("二维码上方 hand 只能是 holding/empty")
        out["hand"] = hand
        out["path"] = str(step.get("path") or "").strip()
    elif kind == "hand_grasp":
        pass
    elif kind == "hand_grasp_factory":
        pass
    elif kind == "vision_place":
        pass
    elif kind == "head":
        out["m0"] = float(step.get("m0") if step.get("m0") not in (None, "") else 0.0)
        out["m1"] = float(step.get("m1") if step.get("m1") not in (None, "") else 40.0)
        out["m2"] = float(step.get("m2") if step.get("m2") not in (None, "") else 0.0)
    elif kind == "modbus_wait":
        signal = int(step.get("signal") if step.get("signal") not in (None, "") else 10)
        if signal not in (10, 20):
            raise ValueError("等待信号只能是 10（可上料）或 20（可下料）")
        out["signal"] = signal
        out["timeout_sec"] = float(step.get("timeout_sec") if step.get("timeout_sec") not in (None, "") else 120)
        if out["timeout_sec"] <= 0:
            raise ValueError("等待超时必须 > 0")
        out["host"] = str(step.get("host") or "").strip()
        addr = step.get("addr")
        out["addr"] = int(addr) if addr not in (None, "", 0, "0") else 0
    elif kind == "modbus_send":
        if step.get("value") in (None, ""):
            raise ValueError("发送机床信号需要填写数值")
        out["value"] = int(step["value"])
        out["host"] = str(step.get("host") or "").strip()
        addr = step.get("addr")
        out["addr"] = int(addr) if addr not in (None, "", 0, "0") else 0
    elif kind.startswith("chassis"):
        raise ValueError("底盘动作已禁用（底盘后期更换，本工具不编排底盘）")
    else:
        raise ValueError(f"未知动作类型: {kind}")
    return out


def program_document(name: str, steps: list[dict[str, Any]]) -> dict[str, Any]:
    clean = [canonicalize_step(s) for s in steps]
    return {
        "schema": SCHEMA_VERSION,
        "name": name,
        "steps": clean,
        # 底盘字段预留但不使用
        "chassis": None,
    }


def validate_program(steps: list[dict[str, Any]]) -> list[str]:
    """返回错误列表；空列表表示可运行。"""
    errors: list[str] = []
    if not steps:
        errors.append("序列为空")
        return errors
    try:
        clean = [canonicalize_step(s) for s in steps]
    except ValueError as exc:
        errors.append(str(exc))
        return errors

    for i, step in enumerate(clean, 1):
        if step["kind"] == "arm" and step.get("left") and step.get("right") is None:
            pass
        if step["kind"] == "arm" and step.get("right") and step.get("left") is None:
            pass

    # 同一步内不可能双左；检查「相邻两步同为仅左或仅右」——允许（两次移动）。
    # 用户说的重复添加：在 merge_into_sequence 阶段拦截。
    return errors


def _arm_side_set(step: dict[str, Any]) -> set[str]:
    sides = set()
    if step.get("left"):
        sides.add("left")
    if step.get("right"):
        sides.add("right")
    return sides


def _grip_side_set(step: dict[str, Any]) -> set[str]:
    sides = set()
    if step.get("left"):
        sides.add("left")
    if step.get("right"):
        sides.add("right")
    return sides


def merge_into_sequence(
    steps: list[dict[str, Any]],
    incoming: dict[str, Any],
    *,
    target_id: str | None = None,
) -> tuple[list[dict[str, Any]], str, bool]:
    """把库里拖来的动作合进序列。

    返回 (新序列, 提示, 是否合并进已有步)。
    同侧重复 → ValueError。
    """
    incoming = canonicalize_step(incoming)
    seq = [canonicalize_step(s) for s in steps]

    target = None
    if target_id:
        target = next((s for s in seq if s["id"] == target_id), None)
    if target is None and seq:
        target = seq[-1]

    if incoming["kind"] == "arm":
        in_sides = _arm_side_set(incoming)
        if target is not None and target["kind"] == "arm" and in_sides:
            have = _arm_side_set(target)
            fresh = in_sides - have
            if not fresh:
                seq.append(incoming)
                return seq, "当前步该侧手臂已有，已新建下一步（同一步不能两个左臂/右臂）", False
            if "left" in fresh:
                target["left"] = incoming["left"]
            if "right" in fresh:
                target["right"] = incoming["right"]
            if incoming.get("speed"):
                target["speed"] = incoming["speed"]
            seq = [canonicalize_step(s) if s["id"] != target["id"] else canonicalize_step(target) for s in seq]
            return seq, "已合并为双臂动作（将双线程执行）", True
        seq.append(incoming)
        return seq, "已添加手臂动作", False

    if incoming["kind"] == "gripper":
        in_sides = _grip_side_set(incoming)
        if target is not None and target["kind"] == "gripper" and in_sides:
            have = _grip_side_set(target)
            fresh = in_sides - have
            if not fresh:
                seq.append(incoming)
                return seq, "当前步该侧夹爪已有，已新建下一步", False
            if "left" in fresh:
                target["left"] = incoming["left"]
            if "right" in fresh:
                target["right"] = incoming["right"]
            seq = [canonicalize_step(s) if s["id"] != target["id"] else canonicalize_step(target) for s in seq]
            return seq, "已合并为双爪动作（将双线程执行）", True
        seq.append(incoming)
        return seq, "已添加夹爪动作", False

    seq.append(incoming)
    if incoming["kind"] == "conveyor_goto":
        side_t = "右侧" if incoming.get("side") == "right" else "左侧"
        arm = incoming.get("arm") or "auto"
        if arm in ("left", "right"):
            arm_t = "左臂" if arm == "left" else "右臂"
            extra = f"+{incoming['path']}" if incoming.get("path") else ""
            return seq, f"已添加传送带{side_t}上方({arm_t}{extra})", False
        hand_t = "持料手" if incoming.get("hand") == "holding" else "空手"
        extra = f"+{incoming['path']}" if incoming.get("path") else ""
        return seq, f"已添加传送带{side_t}上方({hand_t}{extra})", False
    if incoming["kind"] == "aruco_above":
        arm = incoming.get("arm") or "auto"
        if arm in ("left", "right"):
            arm_t = "左臂" if arm == "left" else "右臂"
            return seq, f"已添加二维码上方({arm_t})", False
        hand_t = "持料手" if incoming.get("hand") == "holding" else "空手"
        return seq, f"已添加二维码上方({hand_t})", False
    if incoming["kind"] == "modbus_wait":
        return seq, f"已添加等待信号{incoming.get('signal')}", False
    if incoming["kind"] == "modbus_send":
        return seq, f"已添加发送机床信号 {incoming.get('value')}", False
    titles = {item["seed"]["kind"]: item["title"] for item in LIBRARY}
    return seq, f"已添加{titles.get(incoming['kind'], incoming['kind'])}", False


def step_label(step: dict[str, Any]) -> str:
    kind = step.get("kind")
    if kind == "arm":
        sides = []
        if step.get("left"):
            sides.append("左")
        if step.get("right"):
            sides.append("右")
        if len(sides) == 2:
            return "双臂直线"
        if sides == ["左"]:
            return "左臂直线"
        return "右臂直线"
    if kind == "gripper":
        bits = []
        if step.get("left"):
            bits.append("左" + ("开" if step["left"] == "open" else "握"))
        if step.get("right"):
            bits.append("右" + ("开" if step["right"] == "open" else "握"))
        return "夹爪 " + " / ".join(bits)
    if kind == "waist":
        mode = step.get("mode")
        names = {"rotate": "转腰", "fold": "腰部折叠", "lift": "升降", "advance": "前进", "goto": "腰到坐标"}
        return names.get(mode, "腰部")
    if kind == "conveyor_goto":
        side_t = "右侧" if step.get("side") == "right" else "左侧"
        arm = step.get("arm") or "auto"
        if arm in ("left", "right"):
            base = f"传送带{side_t}上方({ '左臂' if arm == 'left' else '右臂' })"
        else:
            hand_t = "持料手" if step.get("hand") == "holding" else "空手"
            base = f"传送带{side_t}上方({hand_t})"
        if step.get("path"):
            return base + f" · {step['path']}"
        return base
    if kind == "aruco_above":
        arm = step.get("arm") or "auto"
        if arm in ("left", "right"):
            return f"二维码上方({ '左臂' if arm == 'left' else '右臂' })"
        hand_t = "持料手" if step.get("hand") == "holding" else "空手"
        return f"二维码上方({hand_t})"
    names = {
        "sleep": "等待",
        "home": "双手回初始",
        "vision_grasp": "视觉抓取",
        "vision_detect": "视觉检测",
        "vision_grasp_factory": "视觉抓取(新模型)",
        "vision_detect_jindi": "视觉检测(金帝四类)",
        "vision_grasp_jindi": "视觉抓取(金帝·毛胚)",
        "aruco_detect": "二维码位姿",
        "conveyor_goto": "传送带上方",
        "aruco_above": "二维码上方",
        "hand_grasp": "手部相机识别和抓取",
        "hand_grasp_factory": "手部相机识别和抓取(小毛胚)",
        "vision_place": "视觉放货",
        "head": "头部姿态",
        "modbus_wait": "等待机床信号",
        "modbus_send": "发送机床信号",
    }
    if kind == "modbus_wait":
        sig = step.get("signal")
        if sig == 10:
            return "等待上料许可(10)"
        if sig == 20:
            return "等待下料许可(20)"
        return f"等待机床信号({sig})"
    if kind == "modbus_send":
        return f"发送机床信号({step.get('value')})"
    return names.get(kind, kind)


LAYER3_HOME_POSE_OLD = {"x": -0.132502, "y": 0.0, "z": 0.642464, "rx": -90.0, "ry": -90.0, "rz": 180.0}
LAYER3_HOME_POSE = {"x": 0.132502, "y": 0.0, "z": 0.662418, "rx": -90.0, "ry": -90.0, "rz": 180.0}


def default_boot_steps() -> list[dict[str, Any]]:
    """与原先 orch_hw init_hardware 里写死的启动姿态对齐（不含夹爪/相机/CAN 上电）。"""
    return [
        canonicalize_step(
            {
                "kind": "head",
                "m0": 0.0,
                "m1": 40.0,
                "m2": 0.0,
                "note": "启动：头 M0=0° M1=40° M2=0°",
            }
        ),
        canonicalize_step(
            {
                "kind": "waist",
                "mode": "rotate",
                "angle_deg": 0.0,
                "speed_deg_s": 30.0,
                "note": "启动：腰 yaw=0°",
            }
        ),
        canonicalize_step({"kind": "sleep", "seconds": 1.0, "note": "转腰后等待"}),
        canonicalize_step(
            {
                "kind": "waist",
                "mode": "goto",
                "pose": dict(LAYER3_HOME_POSE),
                "speed": 0.1,
                "note": "启动：腰到 layer3_home",
            }
        ),
        canonicalize_step({"kind": "sleep", "seconds": 2.0, "note": "腰到位等待"}),
        canonicalize_step(
            {
                "kind": "arm",
                "left": dict(DEFAULT_POSE_LEFT),
                "right": dict(DEFAULT_POSE_RIGHT),
                "speed": 0.2,
                "note": "启动：双臂待机",
            }
        ),
    ]
