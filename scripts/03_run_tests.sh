#!/usr/bin/env bash
set -euo pipefail

# Log helper that writes to stderr so it doesn't interfere with stdout data capture needed for CI
log() { echo -e "$*" >&2; }

PYTHON_BIN="${PYTHON_BIN:?Error: PYTHON_BIN not set}"
TEST_DIR="tests"

echo ${PYTHON_BIN}

log "Running tests in: $TEST_DIR"
if [[ ! -d "$TEST_DIR" ]]; then
    log "❌ Test directory '$TEST_DIR' not found."
    exit 1
fi

"${PYTHON_BIN}" -m pytest "$TEST_DIR"
