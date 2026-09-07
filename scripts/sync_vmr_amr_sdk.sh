#!/bin/bash
# 将 VMR_AMR_SDK 头文件与 RPC 动态库同步到工程 sdk/ 目录（libvmr_sdk 仍保留在 sdk/lib）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SDK="${ROOT}/VMR_AMR_SDK"
DST_INC="${ROOT}/sdk/include"
DST_LIB="${ROOT}/sdk/lib"
ARCH="$(uname -m)"
case "$ARCH" in
  aarch64|arm64) LIB_ARCH=arm64 ;;
  x86_64|i686) LIB_ARCH=x86 ;;
  *) echo "unsupported arch: $ARCH"; exit 1 ;;
esac
UBUNTU_VER=""
if [ -r /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  UBUNTU_VER="${VERSION_ID:-}"
fi
LIB_SRC="${SDK}/lib/${LIB_ARCH}"
if [ -n "$UBUNTU_VER" ] && [ -d "${LIB_SRC}/${UBUNTU_VER}" ]; then
  LIB_SRC="${LIB_SRC}/${UBUNTU_VER}"
fi

echo "[sync] headers -> ${DST_INC}"
rsync -a "${SDK}/include/" "${DST_INC}/"

# 恢复 VMR 点导航头文件（VMR_AMR_SDK 包不含 VmrSDKApi）
for f in VmrSDKApi.h VmrDefine.h VmrSDK_C.h; do
  if [ ! -f "${DST_INC}/${f}" ]; then
    BAK="${ROOT}/../工厂备份/t170c_move_工厂备份1_1/market_simple_2026_1_21_using/sdk/include/${f}"
    if [ -f "$BAK" ]; then
      cp -f "$BAK" "${DST_INC}/${f}"
      echo "[sync] restored ${f} from backup"
    fi
  fi
done

echo "[sync] libclient*.so from ${LIB_SRC} -> ${DST_LIB}"
cp -f "${LIB_SRC}"/libclient_*.so "${LIB_SRC}"/libcmd_promise_manager.so "${DST_LIB}/"
echo "[sync] done"
