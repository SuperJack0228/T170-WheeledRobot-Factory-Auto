#!/bin/bash
# Web code is unchanged. It must share the C++ process's resolved runtime.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${T170_RUNTIME_DIR:-${T170_ROOT:-}}" ]]; then
  echo 'Set T170_RUNTIME_DIR to the runtime printed by the factory launcher, or prepare one with tools/resolve_deployment.py.' >&2
  exit 2
fi
source "$SCRIPT_DIR/runtime_context.sh"
if [[ ! -f "$T170_CONFIG_DIR/move_box_params.yaml" ]]; then
  echo 'Web requires a resolved runtime with configuration.' >&2
  exit 2
fi
export T170_DEPLOYMENT="${T170_DEPLOYMENT:-selected-runtime}"
cd -- "$T170_ROOT"
exec "${T170_PYTHON:-python3}" "$SCRIPT_DIR/../web_debug/server.py" "$@"
