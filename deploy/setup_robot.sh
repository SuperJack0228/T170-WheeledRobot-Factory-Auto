#!/bin/bash
set -euo pipefail
package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${T170_PYTHON:-python3}" "$package_root/tools/setup_robot.py" "$@"
