#!/usr/bin/env bash
set -euo pipefail

FREECAD_VERSION="${1:-1.1.1}"
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
DOWNLOAD_DIR="$(dirname ${SCRIPT_DIR})/freecad_versions"

export FREECAD_VERSION
export DOWNLOAD_DIR

echo "=== FreeCAD Testing Pipeline ==="

eval "$(bash ${SCRIPT_DIR}/01_download_and_extract.sh)"
export EXTRACT_DIR=${EXTRACT_DIR}

eval "$(bash ${SCRIPT_DIR}/02_setup_env.sh)"
export PYTHON_BIN=${PYTHON_BIN}
export PATH_TO_FREECAD_LIBDIR=${FREECAD_LIB_DIR}

bash ${SCRIPT_DIR}/03_run_tests.sh

echo "=== Pipeline Complete ==="
