#!/bin/bash
# Sourced by launchers. A resolved runtime is data, not a second source checkout.
set -euo pipefail
T170_SOURCE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${T170_RUNTIME_DIR:-}" ]]; then
  # Assignment must fail before export: `export X=$(cd ...)` hides cd's exit code.
  T170_ROOT="$(cd -- "$T170_RUNTIME_DIR" && pwd)" || exit 1
  export T170_ROOT
fi
export T170_ROOT="${T170_ROOT:-${MARKET_SIMPLE_ROOT:-$T170_SOURCE_ROOT}}"
T170_ROOT="$(cd -- "$T170_ROOT" && pwd)"
export T170_ROOT
if [[ -f "$T170_ROOT/config/move_box_params.yaml" ]]; then
  export T170_CONFIG_DIR="${T170_CONFIG_DIR:-$T170_ROOT/config}"
elif [[ -f "$T170_ROOT/config/deployment.yaml" ]]; then
  source_root="$T170_ROOT"
  python_bin="${T170_PYTHON:-python3}"
  manifest="${T170_DEPLOYMENT:-}"
  if [[ -z "$manifest" ]]; then
    manifest="$("$python_bin" - "$source_root" <<'PY'
from pathlib import Path
import sys,yaml
root=Path(sys.argv[1]);selector=root/'config/deployment.yaml'
print((selector.parent/yaml.safe_load(selector.read_text())['manifest']).resolve())
PY
)"
  fi
  state_root="${T170_STATE_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/t170}"
  mkdir -p -- "$state_root"
  session_dir="$(mktemp -d "$state_root/session.XXXXXXXX")"
  runtime="$session_dir/runtime"
  "$python_bin" "$source_root/tools/resolve_deployment.py" "$manifest" "$runtime" --root "$source_root"
  export T170_ROOT="$runtime" T170_CONFIG_DIR="$runtime/config" T170_DEPLOYMENT="$manifest"
  "$source_root/build_robot/t170_config_dump" "$runtime/config/move_box_params.yaml" "$runtime/effective_config.yaml"
  echo "[deployment] T170_RUNTIME_DIR=$runtime" >&2
fi
export T170_RUNTIME_DIR="$T170_ROOT"
# Prefer the library shipped with this build over another libti5_robot.so in an
# inherited environment. The CMake-selected platform is recorded beside the app.
if [[ -f "$T170_SOURCE_ROOT/build_robot/arm_sdk_platform.txt" ]]; then
  IFS= read -r arm_sdk_platform < "$T170_SOURCE_ROOT/build_robot/arm_sdk_platform.txt"
  arm_sdk_library_dir="$T170_SOURCE_ROOT/dependencies/sdk/arm/lib/$arm_sdk_platform"
  export LD_LIBRARY_PATH="$arm_sdk_library_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
