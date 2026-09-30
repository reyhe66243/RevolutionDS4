#!/bin/sh
# Configure and build the REVOLUTIONDS4 adapter firmware.
#
# The RP2040 SDK location is taken from the PICO_SDK_PATH environment variable
# (or from src/lib/pico-sdk if that directory exists).
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

: "${PICO_SDK_PATH:?set PICO_SDK_PATH to the location of the RP2040 SDK}"

BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/firmware}"
OUT_DIR="${OUT_DIR:-$REPO_ROOT/build}"

mkdir -p "$BUILD_DIR" "$OUT_DIR"

cmake -S "$SCRIPT_DIR/src" -B "$BUILD_DIR" -G "Unix Makefiles" -DPICO_BOARD=pico_w
make -C "$BUILD_DIR" -j"$(nproc)"

if [ -f "$BUILD_DIR/REVOLUTIONDS4.uf2" ]; then
    if [ "$BUILD_DIR" != "$OUT_DIR" ]; then
        cp -f "$BUILD_DIR/REVOLUTIONDS4.uf2" "$OUT_DIR/REVOLUTIONDS4.uf2"
    fi
    if [ "$BUILD_DIR" != "$REPO_ROOT" ] && [ "$OUT_DIR" != "$REPO_ROOT" ]; then
        cp -f "$BUILD_DIR/REVOLUTIONDS4.uf2" "$REPO_ROOT/REVOLUTIONDS4.uf2"
    fi
fi

echo
echo "Firmware built: $OUT_DIR/REVOLUTIONDS4.uf2"
