#!/bin/bash
# Development checkout update/build only. No packaging, tagging or device startup.
set -euo pipefail
T170_DEV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(uname -s)" != Linux ]]; then
  echo '开发更新请在机器人 Linux 主机的源码目录执行；Mac 负责修改和推送代码。' >&2
  exit 2
fi
if [[ ! -d "$T170_DEV_ROOT/.git" && ! -f "$T170_DEV_ROOT/.git" ]]; then
  echo '当前目录是运行包或源码副本。请先 git clone 仓库，再执行开发更新。' >&2
  exit 2
fi
cd -- "$T170_DEV_ROOT"
echo '[dev] 请先退出机器人主程序和网页；本脚本不会启动设备。'
git pull --ff-only
cmake -S "$T170_DEV_ROOT" -B "$T170_DEV_ROOT/build_robot" \
  -DCMAKE_BUILD_TYPE=Release -DT170_SERVO_TESTS_ONLY=OFF -DT170_ARM_SDK_CHECK_ONLY=OFF "$@"
cmake --build "$T170_DEV_ROOT/build_robot" --parallel "${T170_BUILD_JOBS:-3}"
echo "[dev] 更新完成，代码提交：$(git rev-parse --short HEAD)"
echo '[dev] 本机档案未重建。启动维护：./deploy/run_maintenance.sh'
echo '[dev] 另开终端启动网页：./deploy/run_web.sh'
