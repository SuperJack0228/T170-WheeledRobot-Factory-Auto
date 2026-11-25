#!/bin/bash
# Shared source/installed launcher. Runtime snapshots never own persistent settings.
set -euo pipefail
T170_PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${T170_WORKSPACE:-}" ]]; then
  if [[ -f "$T170_PACKAGE_ROOT/release.json" ]]; then
    T170_WORKSPACE="$T170_PACKAGE_ROOT"
  else
    T170_WORKSPACE="${T170_ROOT:-$T170_PACKAGE_ROOT}"
    if [[ -n "${T170_RUNTIME_DIR:-}" && -f "$T170_RUNTIME_DIR/deployment_resolved.json" ]]; then
      T170_WORKSPACE="$("${T170_PYTHON:-python3}" -c 'import json,sys;print(json.load(open(sys.argv[1]))["resource_root"])' "$T170_RUNTIME_DIR/deployment_resolved.json")"
    fi
  fi
fi
export T170_WORKSPACE
T170_SOURCE_ROOT="$(cd -- "$T170_WORKSPACE" && pwd)"
echo "[deployment] T170_WORKSPACE=$T170_SOURCE_ROOT (config: $T170_SOURCE_ROOT/config)" >&2
T170_BINARY_ROOT="${T170_BINARY_ROOT:-}"
if [[ -z "$T170_BINARY_ROOT" ]]; then
  for candidate in "$T170_PACKAGE_ROOT/bin" "$T170_PACKAGE_ROOT/build_robot" "$T170_PACKAGE_ROOT/build_release" "$T170_SOURCE_ROOT/bin" "$T170_SOURCE_ROOT/build_robot"; do
    if [[ -f "$candidate/t170_factory" ]]; then T170_BINARY_ROOT="$candidate"; break; fi
  done
fi
export T170_BINARY_ROOT
if [[ -n "${T170_RUNTIME_DIR:-}" ]]; then
  T170_ROOT="$(cd -- "$T170_RUNTIME_DIR" && pwd)" || exit 1
fi
export T170_ROOT="${T170_ROOT:-${MARKET_SIMPLE_ROOT:-$T170_SOURCE_ROOT}}"
T170_ROOT="$(cd -- "$T170_ROOT" && pwd)"
export T170_ROOT
state_root="${T170_STATE_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/t170}"
if [[ -f "$T170_ROOT/config/effective_params.yaml" ]]; then
  export T170_CONFIG_DIR="${T170_CONFIG_DIR:-$T170_ROOT/config}"
elif [[ -f "$T170_ROOT/config/deployment.yaml" ]]; then
  python_bin="${T170_PYTHON:-python3}"
  manifest="${T170_DEPLOYMENT:-}"
  if [[ -z "$manifest" ]]; then
    manifest="$("$python_bin" - "$T170_SOURCE_ROOT" <<'PY'
from pathlib import Path
import sys,yaml
root=Path(sys.argv[1]);value=yaml.safe_load((root/'config/deployment.yaml').read_text()).get('manifest')
if not value:raise SystemExit('请先运行 deploy/setup_robot.sh 建立机器人档案')
print((root/'config'/value).resolve())
PY
)"
  fi
  mkdir -p -- "$state_root"
  session_dir="$(mktemp -d "$state_root/session.XXXXXXXX")"
  runtime="$session_dir/runtime"
  "$python_bin" "$T170_PACKAGE_ROOT/tools/resolve_deployment.py" "$manifest" "$runtime" --root "$T170_SOURCE_ROOT"
  export T170_ROOT="$runtime" T170_CONFIG_DIR="$runtime/config" T170_DEPLOYMENT="$manifest"
  "$T170_BINARY_ROOT/t170_config_dump" "$runtime/config/effective_params.yaml" "$runtime/effective_config.yaml"
  echo "[deployment] T170_RUNTIME_DIR=$runtime" >&2
fi
export T170_RUNTIME_DIR="$T170_ROOT"
if [[ -z "${T170_LOG_DIR:-}" && -f "${T170_CONFIG_DIR:-$T170_ROOT/config}/effective_params.yaml" ]]; then
  export T170_LOG_DIR="$state_root/logs/$(date +%Y%m%d)"
fi
if [[ -n "${T170_LOG_DIR:-}" ]]; then
  mkdir -p -- "$T170_LOG_DIR"
  if [[ -d "$T170_ROOT/logs" && ! -L "$T170_ROOT/logs" ]]; then rmdir -- "$T170_ROOT/logs" 2>/dev/null || true; fi
  if [[ ! -e "$T170_ROOT/logs" ]]; then ln -s -- "$T170_LOG_DIR" "$T170_ROOT/logs"; fi
  echo "[deployment] T170_LOG_DIR=$T170_LOG_DIR" >&2
fi
t170_exec_logged() {
  if [[ -n "${T170_LOG_DIR:-}" ]]; then
    exec > >(tee -a -- "$T170_LOG_DIR/startup.log") 2>&1
    echo "----- $(date '+%Y-%m-%dT%H:%M:%S%z') -----"
  fi
  exec "$@"
}
if [[ -d "$T170_PACKAGE_ROOT/lib" ]]; then
  export LD_LIBRARY_PATH="$T170_PACKAGE_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
elif [[ -f "$T170_BINARY_ROOT/arm_sdk_platform.txt" ]]; then
  IFS= read -r arm_sdk_platform < "$T170_BINARY_ROOT/arm_sdk_platform.txt"
  export LD_LIBRARY_PATH="$T170_PACKAGE_ROOT/dependencies/sdk/arm/lib/$arm_sdk_platform${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
