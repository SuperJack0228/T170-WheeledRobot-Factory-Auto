#!/usr/bin/env bash
# 新机器安装 libserialport（夹爪串口依赖）
set -euo pipefail
DEB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo dpkg -i "${DEB_DIR}"/libserialport0_*.deb "${DEB_DIR}"/libserialport-dev_*.deb
sudo ldconfig
echo "libserialport 安装完成"
