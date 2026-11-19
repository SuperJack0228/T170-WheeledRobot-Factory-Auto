#!/usr/bin/env python3
"""Print the live factory process's runtime and configuration directories, one per line."""
import argparse
import json
from pathlib import Path
import socket
import sys


def discover(host, port):
    with socket.create_connection((host, port), timeout=3.0) as connection:
        connection.settimeout(3.0)
        connection.sendall(b'{"cmd":"ping"}\n')
        response = b''
        while b'\n' not in response:
            block = connection.recv(4096)
            if not block:
                break
            response += block
            if len(response) > 65536:
                raise ValueError('主程序响应过长')
    data = json.loads(response.partition(b'\n')[0])
    if not isinstance(data, dict) or data.get('ok') is not True:
        raise ValueError('主程序没有返回有效的 ping 结果')
    runtime = data.get('runtime_root')
    if not isinstance(runtime, str) or not runtime:
        raise ValueError('当前主程序未提供运行目录，请重新编译并重启，或显式设置 T170_RUNTIME_DIR')
    config = data.get('config_dir', str(Path(runtime) / 'config'))
    for value in (runtime, config):
        if not isinstance(value, str) or '\n' in value or '\r' in value or not Path(value).is_absolute():
            raise ValueError('主程序返回的目录格式无效')
    if not Path(runtime).is_dir() or not (Path(config) / 'move_box_params.yaml').is_file():
        raise ValueError('主程序返回的运行配置在本机不存在')
    return runtime, config


def main():
    # Accept Web's arguments without consuming them; the shell forwards the
    # original arguments intact to server.py. --port is the WEB port, not RPC.
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument('--robot-host', default='127.0.0.1')
    parser.add_argument('--robot-port', type=int, default=8099)
    args, _ = parser.parse_known_args()
    if not 1 <= args.robot_port <= 65535:
        parser.error('--robot-port must be 1..65535')
    try:
        runtime, config = discover(args.robot_host, args.robot_port)
    except (OSError, ValueError) as error:
        print('[web] 无法确定当前机器人运行目录: '+str(error), file=sys.stderr)
        print('[web] 请先启动机器人服务；已有运行目录也可用 T170_RUNTIME_DIR 显式指定。', file=sys.stderr)
        return 2
    print(runtime)
    print(config)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
