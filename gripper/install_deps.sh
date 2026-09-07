#!/usr/bin/env bash
# 安装 DEX1 夹爪直接串口控制依赖（与主工程 third_party/debs 一致）
set -euo pipefail

GRIPPER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== [1/2] 安装 libserialport ==="
DEB_DIR="${GRIPPER_DIR}/third_party/debs"
if [ ! -d "${DEB_DIR}" ]; then
  DEB_DIR="${GRIPPER_DIR}/../third_party/debs"
fi

if [ -f "${DEB_DIR}/install_libserialport.sh" ]; then
  sudo bash "${DEB_DIR}/install_libserialport.sh"
else
  sudo apt-get update
  sudo apt-get install -y git cmake g++ build-essential libserialport-dev
fi
sudo ldconfig

echo ""
echo "=== [2/2] 同步 vendor 并编译 ==="
if [ ! -d "${GRIPPER_DIR}/third_party/dex1" ]; then
  "${GRIPPER_DIR}/sync_vendor.sh"
fi
"${GRIPPER_DIR}/build.sh"

echo ""
echo "完成。无需 ROS2 / DDS / dex1_1_gripper_server。"
echo "  sudo ${GRIPPER_DIR}/build/gripper_calibrate"
echo "  sudo ${GRIPPER_DIR}/build/gripper_demo"
