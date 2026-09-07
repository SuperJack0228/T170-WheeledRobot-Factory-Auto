#!/usr/bin/env bash
# 从上级工程同步夹爪依赖（third_party + lib），便于 gripper/ 目录独立移植
set -euo pipefail

GRIPPER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAIN_DIR="$(cd "${GRIPPER_DIR}/.." && pwd)"

echo "=== 同步夹爪依赖: ${MAIN_DIR} -> ${GRIPPER_DIR} ==="

for item in third_party lib; do
  if [ ! -d "${MAIN_DIR}/${item}" ]; then
    echo "错误: 缺少 ${MAIN_DIR}/${item}" >&2
    exit 1
  fi
  rm -rf "${GRIPPER_DIR}/${item}"
  cp -a "${MAIN_DIR}/${item}" "${GRIPPER_DIR}/${item}"
  echo "  已复制 ${item}/"
done

echo ""
echo "完成。gripper/ 现已包含与主工程一致的 SDK 头文件与 libUnitreeMotorSDK_*.so"
echo "  cd gripper && ./build.sh"
