#!/usr/bin/env bash
# 动作编排台（真机，不开仿真）。在「能跑 ./build/move」的同一终端执行。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if [[ ! -x build/orch_hw ]]; then
  echo "请先编译: cmake --build build --target orch_hw -j"
  exit 1
fi
if [[ ! -r /dev/ttyUSB0 ]]; then
  echo "打不开 /dev/ttyUSB0。夹爪串口属于 dialout 组，请先执行："
  echo "  sudo usermod -aG dialout \$USER"
  echo "然后重新登录，或在本终端执行: newgrp dialout"
  echo "再重新运行本脚本。"
  exit 1
fi
export ORCH_MODE=hw
exec python3 orchestrator/server.py --mode hw --host 0.0.0.0 --port 8088
