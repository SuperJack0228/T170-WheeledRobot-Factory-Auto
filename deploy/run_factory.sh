#!/bin/bash
# The source selector or an explicit resolved runtime chooses configuration.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/runtime_context.sh"
cd -- "$T170_ROOT"
t170_exec_logged "$T170_ROOT/build_robot/t170_factory" "$@"
