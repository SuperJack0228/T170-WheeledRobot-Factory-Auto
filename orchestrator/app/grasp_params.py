"""抓取参数：读写 config/move_box_params.yaml 中与视觉抓取相关的标量。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
YAML_PATH = PROJECT_ROOT / "config" / "move_box_params.yaml"

# path: yaml 嵌套键；网页分组 + 说明
FIELDS: list[dict[str, Any]] = [
    {
        "id": "head_x_threshold",
        "path": ["stagger", "head_x_threshold"],
        "group": "腰进（目标太远）",
        "label": "腰进触发距离 x",
        "unit": "m",
        "step": 0.01,
        "hint": "头部分配后，手臂目标 x 大于此值才让腰前进靠近。当前约 0.52 m。",
    },
    {
        "id": "stagger_step_x",
        "path": ["waist", "stagger_step_x"],
        "group": "腰进（目标太远）",
        "label": "腰一进步长",
        "unit": "m",
        "step": 0.001,
        "hint": "腰每次前进的距离。0.0556 m = 5.56 cm。超出阈值 1 步够就走 1 步，最多按剩余预算走 3 步。",
    },
    {
        "id": "stagger_max_steps",
        "path": ["waist", "stagger_max_steps"],
        "group": "腰进（目标太远）",
        "label": "本轮腰进最多步数",
        "unit": "步",
        "step": 1,
        "kind": "int",
        "hint": "左右手共用这一份预算。用完后即使还远也会尝试抓，不再继续腰进。",
    },
    {
        "id": "waist_x_min",
        "path": ["waist", "x_min"],
        "group": "腰行程 / 降腰抓取",
        "label": "腰 x 最小（前进极限）",
        "unit": "m",
        "step": 0.01,
        "hint": "layer3 x 减小=前进。超出此值的腰进会被截断。",
    },
    {
        "id": "waist_x_max",
        "path": ["waist", "x_max"],
        "group": "腰行程 / 降腰抓取",
        "label": "腰 x 最大（后退极限）",
        "unit": "m",
        "step": 0.01,
        "hint": "layer3 x 增大=后退。",
    },
    {
        "id": "waist_z_min",
        "path": ["waist", "z_min"],
        "group": "腰行程 / 降腰抓取",
        "label": "腰 z 最低",
        "unit": "m",
        "step": 0.01,
        "hint": "抓取自动降腰不会低于此值。现用 home 约 0.66。",
    },
    {
        "id": "waist_z_max",
        "path": ["waist", "z_max"],
        "group": "腰行程 / 降腰抓取",
        "label": "腰 z 最高",
        "unit": "m",
        "step": 0.01,
        "hint": "网页升降腰也会被限制在 z_min~z_max。",
    },
    {
        "id": "grasp_object_z_min",
        "path": ["waist", "grasp_object_z_min"],
        "group": "腰行程 / 降腰抓取",
        "label": "物体太低则降腰（手臂系 z）",
        "unit": "m",
        "step": 0.01,
        "hint": "头相机物体 z 低于此值（例如 -0.46 低于 -0.28）时腰下降，让桌面靠近手臂，避免被 z=-0.33 钳位。",
    },
    {
        "id": "grasp_lower_z",
        "path": ["waist", "grasp_lower_z"],
        "group": "腰行程 / 降腰抓取",
        "label": "一次最多降腰",
        "unit": "m",
        "step": 0.01,
        "hint": "0=关闭自动降腰。实际下降还会被腰 z_min 截断，最多试 2 次，每次降完重拍头相机。",
    },
    {
        "id": "hover_above_m",
        "path": ["head_grasp", "hover_above_m"],
        "group": "先到物体上方",
        "label": "物体上方高度",
        "unit": "m",
        "step": 0.001,
        "hint": "头相机识别后，手先到物体正上方：z = 检测 z + 此值。0.08 m = 8 cm。世界 +z 朝上。",
    },
    {
        "id": "goal_z_base",
        "path": ["head_grasp", "goal_z_base"],
        "group": "先到物体上方",
        "label": "备用高度基准 z",
        "unit": "m",
        "step": 0.001,
        "hint": "仅当检测 z 超出 grasp_valid 时使用。备用高度 = 基准 + 再加。",
    },
    {
        "id": "goal_z_extra",
        "path": ["head_grasp", "goal_z_extra"],
        "group": "先到物体上方",
        "label": "备用高度再加",
        "unit": "m",
        "step": 0.001,
        "hint": "检测 z 无效时加在基准 z 上。正常抓取跟检测 z，不走这项。",
    },
    {
        "id": "goal_x_offset",
        "path": ["head_grasp", "goal_x_offset"],
        "group": "先到物体上方",
        "label": "目标 x 补偿",
        "unit": "m",
        "step": 0.001,
        "hint": "头部分配后加在 x 上。负值=再往身体这边收一点。",
    },
    {
        "id": "head_approach_z_descend",
        "path": ["head_grasp", "head_approach_z_descend"],
        "group": "先到物体上方",
        "label": "头粗定位后再下压（未用）",
        "unit": "m",
        "step": 0.001,
        "hint": "旧流程第二下下压。当前视觉抓取已改为一次走到物体上方，此项不生效。",
    },
    {
        "id": "right_rx_deg",
        "path": ["head_grasp", "right_rx_deg"],
        "group": "先到物体上方",
        "label": "右手姿态 rx",
        "unit": "°",
        "step": 1,
        "hint": "右手到物体上方时的欧拉角 rx。默认 -90°。",
    },
    {
        "id": "right_ry_deg",
        "path": ["head_grasp", "right_ry_deg"],
        "group": "先到物体上方",
        "label": "右手姿态 ry",
        "unit": "°",
        "step": 1,
        "hint": "右手到物体上方时的欧拉角 ry。yaml 现为 90°。",
    },
    {
        "id": "right_rz_deg",
        "path": ["head_grasp", "right_rz_deg"],
        "group": "先到物体上方",
        "label": "右手姿态 rz",
        "unit": "°",
        "step": 1,
        "hint": "右手到物体上方时的欧拉角 rz。默认 0°。",
    },
    {
        "id": "left_rx_deg",
        "path": ["head_grasp", "left_rx_deg"],
        "group": "先到物体上方",
        "label": "左手姿态 rx",
        "unit": "°",
        "step": 1,
        "hint": "左手到物体上方时的欧拉角 rx。默认 +90°。",
    },
    {
        "id": "left_ry_deg",
        "path": ["head_grasp", "left_ry_deg"],
        "group": "先到物体上方",
        "label": "左手姿态 ry",
        "unit": "°",
        "step": 1,
        "hint": "左手到物体上方时的欧拉角 ry。yaml 现为 90°。",
    },
    {
        "id": "left_rz_deg",
        "path": ["head_grasp", "left_rz_deg"],
        "group": "先到物体上方",
        "label": "左手姿态 rz",
        "unit": "°",
        "step": 1,
        "hint": "左手到物体上方时的欧拉角 rz。默认 0°。",
    },
    {
        "id": "hand_descend_z",
        "path": ["head_grasp", "hand_descend_z"],
        "group": "手相机精定位 / 夹取",
        "label": "上方到位后再下压",
        "unit": "m",
        "step": 0.001,
        "hint": "手相机改完 xy 后，沿 z 下压再夹。与「物体上方高度」相同则夹取高度约等于检测 z。",
    },
    {
        "id": "lift_after_grasp_z",
        "path": ["head_grasp", "lift_after_grasp_z"],
        "group": "手相机精定位 / 夹取",
        "label": "夹住后抬起",
        "unit": "m",
        "step": 0.001,
        "hint": "夹紧后 z 抬高这么多再回待机。0.05 m = 5 cm。",
    },
    {
        "id": "hand_detect_invalid_redo_max",
        "path": ["head_grasp", "hand_detect_invalid_redo_max"],
        "group": "手相机精定位 / 夹取",
        "label": "手相机无效最多重拍",
        "unit": "次",
        "step": 1,
        "kind": "int",
        "hint": "首次识别无效后再拍的次数，不含第一拍。0=不重拍。",
    },
    {
        "id": "hand_right_ox",
        "path": ["head_grasp", "hand_grasp", "right", "offset_x"],
        "group": "手相机精定位 / 夹取",
        "label": "右手夹取 x 补偿",
        "unit": "m",
        "step": 0.001,
        "hint": "手相机算出的基座坐标上，右手再加的 x。",
    },
    {
        "id": "hand_right_oy",
        "path": ["head_grasp", "hand_grasp", "right", "offset_y"],
        "group": "手相机精定位 / 夹取",
        "label": "右手夹取 y 补偿",
        "unit": "m",
        "step": 0.001,
        "hint": "右手再加的 y。",
    },
    {
        "id": "hand_left_ox",
        "path": ["head_grasp", "hand_grasp", "left", "offset_x"],
        "group": "手相机精定位 / 夹取",
        "label": "左手夹取 x 补偿",
        "unit": "m",
        "step": 0.001,
        "hint": "左手再加的 x。当前 -0.01 m。",
    },
    {
        "id": "hand_left_oy",
        "path": ["head_grasp", "hand_grasp", "left", "offset_y"],
        "group": "手相机精定位 / 夹取",
        "label": "左手夹取 y 补偿",
        "unit": "m",
        "step": 0.001,
        "hint": "左手再加的 y。",
    },
    {
        "id": "y_side_split",
        "path": ["grasp_zone", "y_side_split"],
        "group": "头部分左右区",
        "label": "侧区 |y| 分界",
        "unit": "m",
        "step": 0.01,
        "hint": "y < -此值 给右手侧区；y > 此值 给左手侧区。中间带里 y≤0 给右手、y>0 给左手。两侧区都有目标才会双手同时抓。",
    },
    {
        "id": "zone_y_max_abs",
        "path": ["grasp_zone", "y_max_abs"],
        "group": "头部分左右区",
        "label": "分配时 |y| 上限",
        "unit": "m",
        "step": 0.01,
        "hint": "头相机目标 |y| 超过则丢掉。台面零件大约 ≤0.20 m；太大（如 0.45）会把画面边角的椅子当分给左手。",
    },
    {
        "id": "zone_x_min",
        "path": ["grasp_zone", "x_min"],
        "group": "头部分左右区",
        "label": "分配时 x 下限",
        "unit": "m",
        "step": 0.01,
        "hint": "头相机目标 x 小于则丢掉。与上限一起框住料盘前方工作区。",
    },
    {
        "id": "zone_x_max",
        "path": ["grasp_zone", "x_max"],
        "group": "头部分左右区",
        "label": "分配时 x 上限",
        "unit": "m",
        "step": 0.01,
        "hint": "头相机目标 x 超过则丢掉。",
    },
    {
        "id": "zone_edge_margin",
        "path": ["grasp_zone", "edge_margin_frac"],
        "group": "头部分左右区",
        "label": "画面边缘丢弃比例",
        "unit": "",
        "step": 0.01,
        "hint": "投影点距图像边小于宽/高的此比例则丢。0.08 可滤掉左下角椅子/地板误检。0=关闭。",
    },
    {
        "id": "zone_cam_xy_over_z",
        "path": ["grasp_zone", "cam_xy_over_z_max"],
        "group": "头部分左右区",
        "label": "相机 |xy|/z 上限",
        "unit": "",
        "step": 0.02,
        "hint": "|cx|/cz 或 |cy|/cz 超过则当画面边缘丢掉。台面约 0.1~0.3，边角误检约 0.55。0=关闭。",
    },
    {
        "id": "gv_x_min",
        "path": ["grasp_valid", "x_min"],
        "group": "手臂允许抓的范围",
        "label": "x 最小",
        "unit": "m",
        "step": 0.01,
        "hint": "超出则这只手本段不移动。",
    },
    {
        "id": "gv_x_max",
        "path": ["grasp_valid", "x_max"],
        "group": "手臂允许抓的范围",
        "label": "x 最大",
        "unit": "m",
        "step": 0.01,
        "hint": "超出则这只手本段不移动。",
    },
    {
        "id": "gv_z_min",
        "path": ["grasp_valid", "z_min"],
        "group": "手臂允许抓的范围",
        "label": "z 最小",
        "unit": "m",
        "step": 0.01,
        "hint": "越负越低。超出则跳过这只手。",
    },
    {
        "id": "gv_z_max",
        "path": ["grasp_valid", "z_max"],
        "group": "手臂允许抓的范围",
        "label": "z 最大",
        "unit": "m",
        "step": 0.01,
        "hint": "越接近 0 越高。",
    },
    {
        "id": "gv_right_y_min",
        "path": ["grasp_valid", "right_y_min"],
        "group": "手臂允许抓的范围",
        "label": "右手 y 最小",
        "unit": "m",
        "step": 0.01,
        "hint": "右手工作空间 y 下沿。",
    },
    {
        "id": "gv_right_y_max",
        "path": ["grasp_valid", "right_y_max"],
        "group": "手臂允许抓的范围",
        "label": "右手 y 最大",
        "unit": "m",
        "step": 0.01,
        "hint": "右手工作空间 y 上沿。",
    },
    {
        "id": "gv_left_y_min",
        "path": ["grasp_valid", "left_y_min"],
        "group": "手臂允许抓的范围",
        "label": "左手 y 最小",
        "unit": "m",
        "step": 0.01,
        "hint": "左手工作空间 y 下沿。",
    },
    {
        "id": "gv_left_y_max",
        "path": ["grasp_valid", "left_y_max"],
        "group": "手臂允许抓的范围",
        "label": "左手 y 最大",
        "unit": "m",
        "step": 0.01,
        "hint": "左手工作空间 y 上沿。",
    },
    {
        "id": "vis_head",
        "path": ["vision_detect", "head_grasp"],
        "group": "识别算法",
        "label": "头相机算法",
        "unit": "",
        "step": 1,
        "kind": "int",
        "hint": "0=2D PnP（分割+椭圆，主要用彩色图）；1=深度重心；-1=跟算法配置走。抓取用 class0。",
    },
    {
        "id": "vis_right",
        "path": ["vision_detect", "right_hand_grasp"],
        "group": "识别算法",
        "label": "右手相机算法",
        "unit": "",
        "step": 1,
        "kind": "int",
        "hint": "同上。右手 D405 精定位。",
    },
    {
        "id": "vis_left",
        "path": ["vision_detect", "left_hand_grasp"],
        "group": "识别算法",
        "label": "左手相机算法",
        "unit": "",
        "step": 1,
        "kind": "int",
        "hint": "同上。左手 D405 精定位。",
    },
    {
        "id": "conv_right_off",
        "path": ["conveyor", "right_offset_m"],
        "group": "传送带（相对二维码）",
        "label": "右侧距离",
        "unit": "m",
        "step": 0.01,
        "hint": "沿二维码印刷 +X（码的右侧）水平挪这么远，右臂走到该点上方。码要贴成 +X 指向右侧传送带。",
    },
    {
        "id": "conv_left_off",
        "path": ["conveyor", "left_offset_m"],
        "group": "传送带（相对二维码）",
        "label": "左侧距离",
        "unit": "m",
        "step": 0.01,
        "hint": "沿二维码印刷 -X（码的左侧）水平挪这么远，左臂走到该点上方。",
    },
    {
        "id": "conv_height",
        "path": ["conveyor", "height_above_m"],
        "group": "传送带（相对二维码）",
        "label": "上方抬高",
        "unit": "m",
        "step": 0.01,
        "hint": "沿码平面法向（朝相机）抬高。要给手相机留工作距离，后续下压量仍用「手部下压」。",
    },
    {
        "id": "conv_above",
        "path": ["conveyor", "above_height_m"],
        "group": "传送带（相对二维码）",
        "label": "码正上方距离",
        "unit": "m",
        "step": 0.005,
        "hint": "「二维码上方」动作：沿码法向 +Z 抬高，无左右偏移。默认 0.05（5cm）。",
    },
    {
        "id": "conv_speed",
        "path": ["conveyor", "speed"],
        "group": "传送带（相对二维码）",
        "label": "手臂速度",
        "unit": "m/s",
        "step": 0.01,
        "hint": "走到传送带上方时的笛卡尔直线速度。",
    },
    {
        "id": "ik_method",
        "path": ["ik", "method"],
        "group": "逆解（笛卡尔直线）",
        "label": "IK 解法",
        "unit": "",
        "kind": "enum",
        "options": [
            {"value": "analytic", "label": "解析（约束 J2）"},
            {"value": "hybrid", "label": "混合（数值估 J2 + 解析）"},
            {"value": "numeric", "label": "数值（阻尼最小二乘）"},
        ],
        "hint": "抓取直线不锁 J2，逆解自己选肘，先保证能到识别点。网页 J2 设定角暂不参与抓取。",
    },
    {
        "id": "ik_j2_from_current",
        "path": ["ik", "j2_from_current"],
        "group": "逆解（笛卡尔直线）",
        "label": "J2 来源",
        "unit": "",
        "kind": "enum",
        "options": [
            {"value": "0", "label": "用下面设定角（到识别点时插值）"},
            {"value": "1", "label": "锁当前角"},
        ],
        "hint": "抓取暂不锁 J2。此项和下面的左右设定角留着以后避障用。",
    },
    {
        "id": "ik_j2_right",
        "path": ["ik", "j2_right_deg"],
        "group": "逆解（笛卡尔直线）",
        "label": "右臂 J2",
        "unit": "电机°",
        "step": 1,
        "hint": "T170 右 J2 大约 0~85°。抓取直线当前不锁此轴。",
    },
    {
        "id": "ik_j2_left",
        "path": ["ik", "j2_left_deg"],
        "group": "逆解（笛卡尔直线）",
        "label": "左臂 J2",
        "unit": "电机°",
        "step": 1,
        "hint": "T170 左 J2 大约 -85~0°。抓取直线当前不锁此轴。",
    },
]


def _indent_of(line: str) -> int:
    return len(line) - len(line.lstrip(" "))


def _key_of(line: str) -> str | None:
    s = line.lstrip()
    if not s or s.startswith("#"):
        return None
    if ":" not in s:
        return None
    return s.split(":", 1)[0].strip()


def _value_span(line: str) -> tuple[int, int] | None:
    """返回「值」在该行的起止下标（不含行尾注释）。"""
    stripped = line.lstrip()
    if ":" not in stripped:
        return None
    after = stripped.split(":", 1)[1]
    # 保留行尾注释
    comment_at = None
    in_str = False
    for i, ch in enumerate(after):
        if ch in "\"'":
            in_str = not in_str
        elif ch == "#" and not in_str:
            comment_at = i
            break
    body = after if comment_at is None else after[:comment_at]
    left = len(body) - len(body.lstrip())
    right = len(body.rstrip())
    if right <= left:
        return None
    base = len(line) - len(stripped) + stripped.index(":") + 1
    return base + left, base + right


def _find_key_line(lines: list[str], path: list[str]) -> int:
    stack: list[tuple[int, str]] = []
    for i, line in enumerate(lines):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        key = _key_of(line)
        if key is None:
            continue
        ind = _indent_of(line)
        while stack and stack[-1][0] >= ind:
            stack.pop()
        stack.append((ind, key))
        if [k for _, k in stack] == path:
            return i
    raise KeyError(".".join(path))


def read_all() -> dict[str, Any]:
    text = YAML_PATH.read_text(encoding="utf-8")
    lines = text.splitlines()
    values: dict[str, Any] = {}
    for field in FIELDS:
        try:
            idx = _find_key_line(lines, field["path"])
            span = _value_span(lines[idx])
            if span is None:
                continue
            raw = lines[idx][span[0] : span[1]].strip()
            if field.get("kind") == "enum":
                values[field["id"]] = raw.strip().strip("\"'")
            elif field.get("kind") == "int":
                values[field["id"]] = int(float(raw))
            else:
                values[field["id"]] = float(raw)
        except (KeyError, ValueError):
            continue
    return values


def schema() -> list[dict[str, Any]]:
    out = []
    for field in FIELDS:
        item = {k: field[k] for k in ("id", "group", "label", "unit", "step", "hint", "options") if k in field}
        item["kind"] = field.get("kind", "float")
        out.append(item)
    return out


def write_values(updates: dict[str, Any]) -> list[str]:
    """按 id 更新 yaml，保留注释。返回改过的 id 列表。"""
    by_id = {f["id"]: f for f in FIELDS}
    text = YAML_PATH.read_text(encoding="utf-8")
    lines = text.splitlines()
    changed: list[str] = []
    for fid, val in updates.items():
        field = by_id.get(fid)
        if field is None:
            continue
        try:
            idx = _find_key_line(lines, field["path"])
        except KeyError:
            continue
        span = _value_span(lines[idx])
        if span is None:
            continue
        if field.get("kind") == "enum":
            new = str(val).strip()
            allowed = {str(o.get("value")) for o in (field.get("options") or [])}
            if allowed and new not in allowed:
                continue
        elif field.get("kind") == "int":
            new = str(int(round(float(val))))
        else:
            num = float(val)
            new = f"{num:.6g}"
        line = lines[idx]
        lines[idx] = line[: span[0]] + new + line[span[1] :]
        changed.append(fid)
    if changed:
        YAML_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return changed


def _fmt_pose6(pose: dict[str, Any]) -> str:
    parts = []
    for key in ("x", "y", "z", "rx", "ry", "rz"):
        num = float(pose[key])
        if abs(num - round(num)) < 1e-9:
            parts.append(str(int(round(num))))
        else:
            parts.append(f"{num:.6g}")
    return "[" + ", ".join(parts) + "]"


def write_standby_poses(left: dict[str, Any] | None, right: dict[str, Any] | None) -> list[str]:
    """把网页启动流程的双手位姿写入 yaml standby，保留注释。"""
    text = YAML_PATH.read_text(encoding="utf-8")
    lines = text.splitlines()
    changed: list[str] = []
    for side, pose in (("right", right), ("left", left)):
        if not pose:
            continue
        try:
            idx = _find_key_line(lines, ["standby", side])
        except KeyError:
            continue
        span = _value_span(lines[idx])
        if span is None:
            continue
        new = _fmt_pose6(pose)
        old = lines[idx][span[0] : span[1]]
        if old.strip() == new:
            continue
        lines[idx] = lines[idx][: span[0]] + new + lines[idx][span[1] :]
        changed.append(side)
    if changed:
        YAML_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return changed


def write_layer3_home(pose: dict[str, Any]) -> bool:
    """把启动流程腰 goto 写入 waist.layer3_home；若还没有 layer3_home_old 则先备份当前值。"""
    text = YAML_PATH.read_text(encoding="utf-8")
    lines = text.splitlines()
    try:
        idx = _find_key_line(lines, ["waist", "layer3_home"])
    except KeyError:
        return False
    span = _value_span(lines[idx])
    if span is None:
        return False
    new = _fmt_pose6(pose)
    old = lines[idx][span[0] : span[1]]
    if old.strip() == new:
        return False
    has_old = False
    try:
        _find_key_line(lines, ["waist", "layer3_home_old"])
        has_old = True
    except KeyError:
        has_old = False
    if not has_old:
        indent = lines[idx][: len(lines[idx]) - len(lines[idx].lstrip(" "))]
        lines.insert(
            idx,
            f"{indent}layer3_home_old: {old.strip()}  # 原初始腰位，未删除",
        )
        idx += 1
        span = _value_span(lines[idx])
        if span is None:
            return False
    lines[idx] = lines[idx][: span[0]] + new + lines[idx][span[1] :]
    YAML_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return True
