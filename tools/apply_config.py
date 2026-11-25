#!/usr/bin/env python3
"""Validate selected persistent YAML and hot-apply when the robot is idle."""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from t170_common.config_workspace import ConfigWorkspace, workspace_root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path)
    parser.add_argument("--robot-host", default="127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=8099)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    def request(payload, timeout=30):
        with socket.create_connection((args.robot_host, args.robot_port), timeout=3) as sock:
            sock.settimeout(timeout); sock.sendall(json.dumps(payload).encode() + b"\n")
            stream = sock.makefile("rb"); response = stream.readline(4 * 1024 * 1024)
            if not response: raise ConnectionError("主程序连接已关闭")
            return json.loads(response)
    try:
        workspace = ConfigWorkspace(args.root or workspace_root())
        result = workspace.apply(None if args.validate_only else request)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr); return 2


if __name__ == "__main__": raise SystemExit(main())
