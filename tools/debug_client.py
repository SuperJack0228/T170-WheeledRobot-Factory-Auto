#!/usr/bin/env python3
"""T170C 无底盘调试客户端。t170c_debug 默认监听 127.0.0.1:8099。"""

import argparse
import json
import socket
import sys

COMMANDS = {
    "ping": {"cmd": "ping"},
    "status": {"cmd": "snapshot"},
    "home": {"cmd": "home"},
    "ready1": {"cmd": "grasp_ready1"},
    "ready2": {"cmd": "grasp_ready2"},
    "ready3": {"cmd": "grasp_ready3"},
    "ready6": {"cmd": "grasp_ready6"},
    "tray2ready1": {"cmd": "tray2ready1"},
    "tray2ready2": {"cmd": "tray2ready2"},
    "tray2ready3": {"cmd": "tray2ready3"},
    "tray2ready6": {"cmd": "tray2ready6"},
    "tray2": {"cmd": "tray2"},
    "tray2_place": {"cmd": "tray2_place"},
    "tray2test": {"cmd": "tray2_precision"},
    "waist1": {"cmd": "waist1"},
    "waist2": {"cmd": "waist2"},
    "belt": {"cmd": "belt"},
    "belt_ready": {"cmd": "belt_ready"},
    "belt_place": {"cmd": "belt_place"},
    "belt_grasp_rpy": {"cmd": "belt_grasp_rpy"},
    "belt_grasp": {"cmd": "belt_grasp"},
    "belt2": {"cmd": "belt2"},
    "belt2_ready": {"cmd": "belt2_ready"},
    "belt2_place": {"cmd": "belt2_place"},
    "belt2_grasp_rpy": {"cmd": "belt2_grasp_rpy"},
    "belt2_grasp": {"cmd": "belt2_grasp"},
    "waist_jog": {"cmd": "waist_jog"},
    "grasp": {"cmd": "vision_grasp"},
    "grasp_belt": {"cmd": "grasp_belt"},
    "cycle": {"cmd": "grasp_belt"},
    "cycle1": {"cmd": "grasp_belt1"},
    "qr": {"cmd": "aruco_detect"},
    "tray": {"cmd": "detect_tray_holes"},
    "abort": {"cmd": "abort"},
    "reload": {"cmd": "reload_config"},
}


def request(payload: dict, host: str, port: int) -> dict:
    with socket.create_connection((host, port), timeout=5) as sock:
        sock.settimeout(1800)
        sock.sendall((json.dumps(payload, ensure_ascii=False) + "\n").encode())
        data = b""
        while not data.endswith(b"\n"):
            chunk = sock.recv(65536)
            if not chunk:
                break
            data += chunk
    return json.loads(data.decode())


def _fmt(v, spec: str = "7.3f") -> str:
    if v is None:
        return "   ---"
    try:
        return format(float(v), spec)
    except (TypeError, ValueError):
        return "   ---"


def _xyz(v) -> str:
    if not isinstance(v, (list, tuple)) or len(v) < 3:
        return "(---, ---, ---)"
    return f"({_fmt(v[0], '8.4f')}, {_fmt(v[1], '8.4f')}, {_fmt(v[2], '8.4f')})"


def print_arm_arrivals(result: dict) -> None:
    rows = result.get("arm_arrivals") or []
    if not rows:
        return
    print(f"手臂到位 {len(rows)} 次（实际点=停稳后编码器正运动学，误差=实际-目标）")
    for i, row in enumerate(rows, 1):
        settled = "停稳" if row.get("settled") else "未停稳"
        print(
            f"{i:3d} {row.get('hand', '?')} {row.get('stage', '')}  {settled}\n"
            f"     目标xyz={_xyz(row.get('goal_xyz'))}\n"
            f"     实际xyz={_xyz(row.get('actual_xyz'))}\n"
            f"     误差xyz={_xyz(row.get('err_xyz'))}  |err|={_fmt(row.get('err_m'), '7.4f')} m"
        )
    print()


def print_tray_table(result: dict) -> None:
    save = result.get("save_path") or ""
    ids = result.get("used_ids") or []
    print(
        f"料盘 {'OK' if result.get('ok_tray') else 'FAIL'}  "
        f"ids={ids}  reproj={_fmt(result.get('reproj_px'), '5.2f')}px"
        f"  tilt={_fmt(result.get('tilt_deg'), '4.1f')}deg"
    )
    if save:
        print(f"标注图: {save}")
        print(
            "看图：品红圈=二维码孔几何；彩色编号圈=1-36；斜十字=YOLO。"
            "圈在真孔上=盘位准；十字落在对应圈内=类别配对准。"
            "绿 raw=毛坯  黄 semi=半成品  蓝 fin=成品  灰 empty=空孔  白=未匹配。"
            "盘面Z=外参斜面；水平盘Z=假定料盘水平后；盘上高=深度相对二维码平面；水平顶Z=水平盘Z+盘上高。"
        )
    holes = result.get("holes") or []
    if not holes:
        return
    print(
        f"{'孔':>3} {'行':>2} {'列':>2} {'类别':<8} {'基座X':>8} {'基座Y':>8} "
        f"{'盘面Z':>8} {'水平盘Z':>8} {'盘上高':>8} {'水平顶Z':>8} {'原顶Z':>8}"
    )
    for h in holes:
        name = h.get("class_name") or "-"
        if h.get("class_id", -1) < 0:
            name = "-"
        print(
            f"{int(h.get('id', 0)):3d} {int(h.get('row', 0)):2d} {int(h.get('col', 0)):2d} "
            f"{str(name):<8} {_fmt(h.get('x'), '8.3f')} {_fmt(h.get('y'), '8.3f')} "
            f"{_fmt(h.get('tray_z'))} {_fmt(h.get('tray_z_level'))} "
            f"{_fmt(h.get('height_on_tray'))} {_fmt(h.get('top_z_level'))} "
            f"{_fmt(h.get('top_z'))}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="T170C 圆柱抓取/头相机二维码调试")
    parser.add_argument("command", choices=COMMANDS)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8099)
    parser.add_argument(
        "--joint",
        type=int,
        default=2,
        help="waist_jog: 1脚踝 2膝盖 3髋 4侧倾 5回转，默认 2",
    )
    parser.add_argument(
        "--dq",
        type=float,
        default=0.05,
        dest="dq_rad",
        help="waist_jog: 模型弧度，默认 0.05（约 2.9°），限幅 ±0.12",
    )
    args = parser.parse_args()
    payload = dict(COMMANDS[args.command])
    if args.command == "waist_jog":
        payload["joint"] = args.joint
        payload["dq_rad"] = args.dq_rad
    try:
        result = request(payload, args.host, args.port)
    except (OSError, ValueError) as exc:
        print(f"请求失败: {exc}", file=sys.stderr)
        return 2
    if args.command == "tray":
        print_tray_table(result)
        print()
    print_arm_arrivals(result)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
