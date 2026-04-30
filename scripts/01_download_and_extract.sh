#!/usr/bin/env bash
set -euo pipefail

# Log helper that writes to stderr so it doesn't interfere with stdout data capture needed for CI
log() { echo -e "$*" >&2; }

FREECAD_VERSION="${1:-${FREECAD_VERSION:?Error: Provide version via \$1 or export FREECAD_VERSION}}"
DOWNLOAD_DIR="${2:-${DOWNLOAD_DIR:?Error: Provide version via \$2 or export DOWNLOAD_DIR}}"
APPIMAGE_FILE="${DOWNLOAD_DIR}/FreeCAD_${FREECAD_VERSION}.AppImage"
EXTRACT_DIR="${DOWNLOAD_DIR}/FreeCAD_${FREECAD_VERSION}"
DEFAULT_VERSION_DIR="${DOWNLOAD_DIR}/FreeCAD_default"


if [[ -d "$EXTRACT_DIR" ]]; then
    log "✅ Already extracted: $EXTRACT_DIR"
    echo "EXTRACT_DIR=$EXTRACT_DIR"
    # Check if symlink already exists and corresponds to the current version
    # Resolve to absolute paths for reliable comparison
    RESOLVED_TARGET=$(readlink -f "$EXTRACT_DIR")
    if [[ -L "$DEFAULT_VERSION_DIR" ]]; then
        CURRENT_TARGET=$(readlink -f "$DEFAULT_VERSION_DIR")
        if [[ "$CURRENT_TARGET" == "$RESOLVED_TARGET" ]]; then
            log "✅ Symlink already points to correct version"
            exit 0
        fi
        log "⚠️ Symlink points to different version: $CURRENT_TARGET; recreating it"
        rm -f "$DEFAULT_VERSION_DIR"
    fi
    # Create symlink
    ln -s ${EXTRACT_DIR} ${DEFAULT_VERSION_DIR}
    exit 0
fi

if [[ -f "$APPIMAGE_FILE" ]]; then
    log "✅ AppImage already downloaded: $APPIMAGE_FILE"
else
    log "Resolving FreeCAD ${FREECAD_VERSION} download URL..."

    # Use GitHub API to find the exact asset (handles -conda, py311/py312, etc.)
    ASSET_URL=$(curl -sL "https://api.github.com/repos/FreeCAD/FreeCAD/releases/tags/${FREECAD_VERSION}" \
        | grep '"browser_download_url":.*Linux-x86_64.*AppImage' \
        | head -1 \
        | sed 's/.*"browser_download_url": "//;s/"$//')

    if [[ -z "$ASSET_URL" ]]; then
        log "❌ Failed getting asset url for downloading the FreeCAD linux appimage..."
        exit 1
    fi

    log "Downloading from: $ASSET_URL"
    if ! curl -L -f -s -o "$APPIMAGE_FILE" "$ASSET_URL"; then
        log "❌ Download failed. Cleaning up partial file..."
        rm -f "$APPIMAGE_FILE"
        exit 1
    fi

    log "✅ Download complete."
fi

log "Extracting to $EXTRACT_DIR..."
chmod +x "$APPIMAGE_FILE"
"$APPIMAGE_FILE" --appimage-extract

if [[ -d "squashfs-root" ]]; then
    mv squashfs-root "$EXTRACT_DIR"
else
    log "❌ Extraction failed: squashfs-root not found"
    exit 1
fi

log "✅ Extraction complete."
