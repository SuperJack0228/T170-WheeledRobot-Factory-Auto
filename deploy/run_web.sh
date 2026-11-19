#!/bin/bash
# Prefer the live factory's runtime. Explicit runtime overrides remain supported.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${T170_RUNTIME_DIR:-}" ]] &&
   { [[ -z "${T170_ROOT:-}" ]] || [[ ! -f "${T170_CONFIG_DIR:-$T170_ROOT/config}/move_box_params.yaml" ]]; }; then
  runtime_info="$("${T170_PYTHON:-python3}" "$SCRIPT_DIR/../tools/active_runtime.py" "$@")"
  export T170_RUNTIME_DIR="${runtime_info%%$'\n'*}"
  export T170_CONFIG_DIR="${runtime_info#*$'\n'}"
  echo "[web] 使用机器人当前运行目录: $T170_RUNTIME_DIR" >&2
fi
source "$SCRIPT_DIR/runtime_context.sh"
if [[ ! -f "$T170_CONFIG_DIR/move_box_params.yaml" ]]; then
  echo 'Web requires a resolved runtime with configuration.' >&2
  exit 2
fi
export T170_DEPLOYMENT="${T170_DEPLOYMENT:-selected-runtime}"
cd -- "$T170_ROOT"
exec "${T170_PYTHON:-python3}" "$SCRIPT_DIR/../web_debug/server.py" "$@"
