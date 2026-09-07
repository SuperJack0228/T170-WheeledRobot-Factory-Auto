#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"
cmake .. "$@"
make -j"$(nproc)"

echo ""
echo "编译完成:"
echo "  标定: ${BUILD_DIR}/gripper_calibrate"
echo "  演示: ${BUILD_DIR}/gripper_demo"
echo "  力矩夹取(经典): ${BUILD_DIR}/gripper_grasp_demo"
echo "  力矩夹取(Soft): ${BUILD_DIR}/gripper_grasp_soft_demo"
echo ""
echo "首次使用（直接串口，无需 dex1_1_gripper_server）:"
echo "  sudo ${BUILD_DIR}/gripper_calibrate"
echo "  sudo ${BUILD_DIR}/gripper_demo"
