#!/usr/bin/env bash
# 硬件模式：先确保 build/orch_hw 已编译。会驱动真机，底盘不参与。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if [[ ! -x build/orch_hw ]]; then
  echo "请先编译: cmake --build build --target orch_hw -j"
  exit 1
fi
export ORCH_MODE=hw
exec python3 orchestrator/server.py --mode hw --host 0.0.0.0 --port 8088
