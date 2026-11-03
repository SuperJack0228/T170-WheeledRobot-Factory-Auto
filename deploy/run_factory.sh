#!/bin/bash
# Foreground launcher only. The caller controls dispatch mode and log capture.
set -euo pipefail
SCRIPT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export T170_ROOT="${T170_ROOT:-${MARKET_SIMPLE_ROOT:-$SCRIPT_ROOT}}"
# Resolve a relative root before changing directory, matching C++/Python tools.
T170_ROOT="$(cd -- "$T170_ROOT" && pwd)"
export T170_ROOT
cd -- "$T170_ROOT"
exec "$T170_ROOT/build_robot/t170_factory" "$@"
