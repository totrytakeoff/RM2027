#!/usr/bin/env bash
# RM2026 scripts/log.sh entry, adapted for the MC-02 framework.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${PYTHON_BIN:-python3}" "$SCRIPT_DIR/devtools.py" log "$@"
