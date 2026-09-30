#!/bin/sh
# Configure and build the REVOLUTIONDS4 IOS module.
#
# The Starlet build needs a big-endian libgcc; pass its directory through the
# REVDS4_LIBGCC_BE environment variable.
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

: "${REVDS4_LIBGCC_BE:?set REVDS4_LIBGCC_BE to the directory containing the big-endian libgcc}"

BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/cios}"
OUT_DIR="${OUT_DIR:-$REPO_ROOT/build}"

mkdir -p "$BUILD_DIR" "$OUT_DIR"

cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -G Ninja \
    -DCMAKE_SYSTEM_NAME=Starlet \
    -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY \
    -DCMAKE_C_COMPILER=arm-none-eabi-gcc \
    -DCMAKE_ASM_COMPILER=arm-none-eabi-gcc \
    -DREVDS4_LIBGCC_BE="$REVDS4_LIBGCC_BE"
ninja -C "$BUILD_DIR"

size=$(stat -c%s "$BUILD_DIR/REVOLUTIONDS4.app")
if [ -f "$BUILD_DIR/REVOLUTIONDS4.app" ]; then
    if [ "$BUILD_DIR" != "$OUT_DIR" ]; then
        cp -f "$BUILD_DIR/REVOLUTIONDS4.app" "$OUT_DIR/REVOLUTIONDS4.app"
    fi
    if [ "$BUILD_DIR" != "$REPO_ROOT" ] && [ "$OUT_DIR" != "$REPO_ROOT" ]; then
        cp -f "$BUILD_DIR/REVOLUTIONDS4.app" "$REPO_ROOT/REVOLUTIONDS4.app"
    fi
fi

echo
echo "Module built: $OUT_DIR/REVOLUTIONDS4.app (${size} bytes)"
[ "$size" -le 36864 ] || { echo "ERROR: module exceeds the 36864 byte limit"; exit 1; }
