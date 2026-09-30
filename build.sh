#!/bin/sh
# Build both REVOLUTIONDS4 firmwares into the top-level build/ directory.
set -e
cd "$(dirname "$0")"

REPO_ROOT="$(pwd)"
ROOT_BUILD_DIR="$REPO_ROOT/build"
mkdir -p "$ROOT_BUILD_DIR"

# 1. Environment auto-detection if not explicitly set
if [ -z "$PICO_SDK_PATH" ]; then
    for candidate in \
        "$REPO_ROOT/firmware/src/lib/pico-sdk" \
        "/media/RAID_Linux/PicoBlåtand/pico-sdk" \
        "$HOME/pico-sdk" \
        "$HOME/pico/pico-sdk"; do
        if [ -f "$candidate/pico_sdk_init.cmake" ]; then
            export PICO_SDK_PATH="$candidate"
            break
        fi
    done
fi

if [ -z "$REVDS4_LIBGCC_BE" ]; then
    for candidate in \
        "$HOME/.local/libgcc-be"/*/thumb/be \
        "$HOME/.local/devkitpro/devkitARM/lib/gcc/arm-none-eabi"/*/thumb/be \
        "${DEVKITPRO:-/opt/devkitpro}/devkitARM/lib/gcc/arm-none-eabi"/*/thumb/be; do
        if [ -f "$candidate/libgcc.a" ]; then
            export REVDS4_LIBGCC_BE="$candidate"
            break
        fi
    done
fi

# Ensure stripios is discoverable in PATH
if ! command -v stripios >/dev/null 2>&1; then
    for bin_candidate in \
        "$HOME/.local/bin" \
        "${DEVKITPRO:-/opt/devkitpro}/tools/bin"; do
        if [ -x "$bin_candidate/stripios" ]; then
            export PATH="$bin_candidate:$PATH"
            break
        fi
    done
fi

TARGET="${1:-all}"

build_firmware() {
    echo "=========================================="
    echo "Building Adapter Firmware (RP2040 / Pico W)"
    echo "=========================================="
    BUILD_DIR="$ROOT_BUILD_DIR/firmware" OUT_DIR="$ROOT_BUILD_DIR" ./firmware/build.sh
}

build_cios() {
    echo "=========================================="
    echo "Building Console Module (Wii cIOS)"
    echo "=========================================="
    BUILD_DIR="$ROOT_BUILD_DIR/cios" OUT_DIR="$ROOT_BUILD_DIR" ./cios/build.sh
}

clean() {
    echo "Cleaning build directory and artifacts..."
    rm -rf "$ROOT_BUILD_DIR" firmware/build cios/build "$REPO_ROOT/REVOLUTIONDS4.uf2" "$REPO_ROOT/REVOLUTIONDS4.app"
    echo "Clean complete."
}

case "$TARGET" in
    all)
        build_firmware
        echo
        build_cios
        echo
        echo "=========================================="
        echo "Build complete! Artifacts available in build/ and root:"
        echo "  - Adapter firmware:  build/REVOLUTIONDS4.uf2"
        echo "  - Wii cIOS module:   build/REVOLUTIONDS4.app ($(stat -c%s "$ROOT_BUILD_DIR/REVOLUTIONDS4.app") bytes)"
        echo "=========================================="
        ;;
    firmware)
        build_firmware
        ;;
    cios)
        build_cios
        ;;
    clean)
        clean
        ;;
    *)
        echo "Usage: $0 [all|firmware|cios|clean]"
        exit 1
        ;;
esac
