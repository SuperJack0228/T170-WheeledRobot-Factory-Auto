#!/bin/bash
# Calibration and manual diagnostics; automatic dispatch is not started at boot.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/run_factory.sh" --maintenance "$@"
