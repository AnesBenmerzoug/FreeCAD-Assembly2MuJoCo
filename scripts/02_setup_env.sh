#!/usr/bin/env bash
set -euo pipefail

# Log helper that writes to stderr so it doesn't interfere with stdout data capture needed for CI
log() { echo -e "$*" >&2; }

EXTRACT_DIR="${1:-${EXTRACT_DIR:?Error: Provide version via \$1 or export EXTRACT_DIR}}"
PYTHON_BIN="${EXTRACT_DIR}/usr/bin/python"

log "Setting up environment..."

"$PYTHON_BIN" -m ensurepip --quiet --upgrade 2>/dev/null || true
"$PYTHON_BIN" -m pip install --quiet -e ".[tests]"

log "✅ Environment ready."
