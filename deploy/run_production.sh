#!/bin/bash
# Factory production entry. No robot/workcell pairing is encoded here.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/run_factory.sh" --production "$@"
