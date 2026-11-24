#!/bin/bash
# Native Linux release build; never starts devices or publishes an artifact.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(uname -s)" != Linux ]]; then echo '运行包必须在目标兼容 Linux 环境构建；Mac 仅运行离线测试。' >&2; exit 2; fi
if [[ -n "$(git -C "$root" status --porcelain)" ]]; then echo '请先提交并冻结候选代码，工作区必须干净。' >&2; exit 2; fi
version="$(cat "$root/packaging/VERSION")"
tag="$(git -C "$root" describe --exact-match --tags HEAD 2>/dev/null || true)"
if [[ "$tag" != "v$version" ]]; then echo "请在当前提交建立候选标签 v$version。" >&2; exit 2; fi
build="${T170_RELEASE_BUILD_DIR:-$root/build_release}"
cmake -S "$root" -B "$build" -DCMAKE_BUILD_TYPE=Release -DT170_SERVO_TESTS_ONLY=OFF -DT170_ARM_SDK_CHECK_ONLY=OFF "$@"
cmake --build "$build" --parallel "${T170_BUILD_JOBS:-3}"
stage="$build/install-check"
cmake --install "$build" --prefix "$stage"
"${T170_PYTHON:-python3}" "$root/packaging/check_runtime.py" "$stage" "$root" "$build/environment.lock.json"
cpack --config "$build/CPackConfig.cmake" -G TGZ -C Release -B "$build/packages"
echo "运行包及 SHA256 位于 $build/packages"
