#!/bin/bash
project_path=$(cd `dirname $0`; pwd)
cd ${project_path}

case $(uname -m) in
    x86_64|i386|i686)
        LIB_ARCH="x86"
        ;;
    aarch64|arm64)
        LIB_ARCH="arm64"
        ;;
    *)
        echo "Unsupported architecture: $(uname -m)"
        exit 1
        ;;
esac

ARCH_LIB_DIR="${project_path}/../../lib/${LIB_ARCH}"
UBUNTU_VERSION=""
if [ -r "${project_path}/build/CMakeCache.txt" ]; then
    CACHE_UBUNTU_VERSION=$(sed -n 's/^SDK_UBUNTU_VERSION:STRING=//p' "${project_path}/build/CMakeCache.txt" | tail -n 1)
    if [ -n "${CACHE_UBUNTU_VERSION}" ]; then
        UBUNTU_VERSION="${CACHE_UBUNTU_VERSION}"
    fi
fi
if [ -r /etc/os-release ]; then
    . /etc/os-release
    if [ -z "${UBUNTU_VERSION}" ]; then
        UBUNTU_VERSION="${VERSION_ID}"
    fi
fi

if [ -n "${UBUNTU_VERSION}" ] && [ -f "${ARCH_LIB_DIR}/${UBUNTU_VERSION}/libclient_internal.so" ]; then
    SDK_LIB_DIR="${ARCH_LIB_DIR}/${UBUNTU_VERSION}"
elif [ -f "${ARCH_LIB_DIR}/libclient_internal.so" ]; then
    SDK_LIB_DIR="${ARCH_LIB_DIR}"
else
    SDK_LIB_DIR=$(find "${ARCH_LIB_DIR}" -mindepth 2 -maxdepth 2 -name libclient_internal.so -printf '%h\n' | sort -V | tail -n 1)
fi

if [ -z "${SDK_LIB_DIR}" ]; then
    echo "No SDK library directory found under ${ARCH_LIB_DIR}"
    exit 1
fi

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${SDK_LIB_DIR}
./build/client_demo "$@"
