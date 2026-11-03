#!/bin/bash
# Compatibility for existing boot/cron installations.
exec "$(dirname -- "${BASH_SOURCE[0]}")/t170_factory_boot.sh" "$@"
