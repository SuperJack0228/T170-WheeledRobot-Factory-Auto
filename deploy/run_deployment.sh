#!/bin/bash
# Explicit source-tree deployment launcher. No fixed workcell/robot pairing.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ "$#" -lt 2 ]; then
  echo "Usage: $0 deployment.yaml NEW_RUNTIME_DIRECTORY [factory arguments]" >&2
  exit 2
fi
manifest="$1"
runtime="$2"
shift 2
manifest="$(cd -- "$(dirname -- "$manifest")" && pwd)/$(basename -- "$manifest")"
python_bin="${T170_PYTHON:-python3}"
"$python_bin" "$ROOT/tools/resolve_deployment.py" "$manifest" "$runtime" --root "${T170_WORKSPACE:-$ROOT}"
runtime="$(cd -- "$runtime" && pwd)"
export T170_WORKSPACE="${T170_WORKSPACE:-$ROOT}"
export T170_ROOT="$runtime" T170_CONFIG_DIR="$runtime/config" T170_DEPLOYMENT="$manifest"
source "$ROOT/deploy/runtime_context.sh"
cd -- "$runtime"
# Validate through the real C++ loader before linking/starting any device SDK.
"$T170_BINARY_ROOT/t170_config_dump" "$runtime/config/effective_params.yaml" "$runtime/effective_config.yaml"
t170_exec_logged "$T170_BINARY_ROOT/t170_factory" "$@"
