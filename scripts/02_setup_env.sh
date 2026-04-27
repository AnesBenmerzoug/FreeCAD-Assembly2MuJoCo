#!/usr/bin/env bash
set -euo pipefail

# Log helper that writes to stderr so it doesn't interfere with stdout data capture needed for CI
log() { echo -e "$*" >&2; }

FREECAD_VERSION="${FREECAD_VERSION:?Error: FREECAD_VERSION not set}"
EXTRACT_DIR="${EXTRACT_DIR:?Error: EXTRACT_DIR not set}"
APP_DIR="${EXTRACT_DIR}/usr"
FREECAD_LIB_DIR="${APP_DIR}/lib"
PYTHON_BIN="${APP_DIR}/bin/python"

log "Setting up environment..."
export PATH_TO_FREECAD_LIBDIR="${FREECAD_LIB_DIR}"
export QT_QPA_PLATFORM=offscreen
export QT_X11_NO_MITSHM=1

log "Installing dependencies"

"$PYTHON_BIN" -m ensurepip --quiet --upgrade 2>/dev/null || true
"$PYTHON_BIN" -m pip install --quiet -e ".[tests]"

log "✅ Environment ready."
echo "PYTHON_BIN=${PYTHON_BIN}"
echo "FREECAD_LIB_DIR=${FREECAD_LIB_DIR}"
