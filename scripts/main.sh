#!/usr/bin/env bash
set -euo pipefail

FREECAD_VERSION="${1:-${FREECAD_VERSION:?Error: Provide version via \$1 or export FREECAD_VERSION}}"
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
DOWNLOAD_DIR="$(dirname ${SCRIPT_DIR})/freecad_versions"
EXTRACT_DIR="${DOWNLOAD_DIR}/FreeCAD_${FREECAD_VERSION}"
PYTHON_BIN="${EXTRACT_DIR}/usr/bin/python"
PATH_TO_FREECAD_LIBDIR="${EXTRACT_DIR}/usr/lib"

echo "=== FreeCAD Testing Pipeline ==="

bash ${SCRIPT_DIR}/01_download_and_extract.sh ${FREECAD_VERSION} ${DOWNLOAD_DIR}

bash ${SCRIPT_DIR}/02_setup_env.sh ${EXTRACT_DIR}

bash ${SCRIPT_DIR}/03_run_tests.sh ${PYTHON_BIN} ${PATH_TO_FREECAD_LIBDIR}

echo "=== Pipeline Complete ==="
