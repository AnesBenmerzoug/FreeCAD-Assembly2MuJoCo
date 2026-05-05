#!/usr/bin/env bash
set -euo pipefail

# Log helper that writes to stderr so it doesn't interfere with stdout data capture needed for CI
log() { echo -e "$*" >&2; }

PYTHON_BIN="${1:-${PYTHON_BIN:?Error: Provide version via \$1 or export PYTHON_BIN}}"
PATH_TO_FREECAD_LIBDIR="${2:-${PATH_TO_FREECAD_LIBDIR:?Error: Provide version via \$2 or export PATH_TO_FREECAD_LIBDIR}}"

export PATH_TO_FREECAD_LIBDIR=${PATH_TO_FREECAD_LIBDIR}
export QT_QPA_PLATFORM=offscreen
export QT_X11_NO_MITSHM=1

# Safely parse extra pytest arguments from env var
PYTEST_EXTRA_ARGS=()
if [[ -n "${PYTEST_ARGS:-}" ]]; then
    eval "set -- $PYTEST_ARGS"
    PYTEST_EXTRA_ARGS=("$@")
fi

"${PYTHON_BIN}" -m pytest "${PYTEST_EXTRA_ARGS[@]}"
