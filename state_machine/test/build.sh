#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_MACHINE_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_DIR="$(dirname "$STATE_MACHINE_DIR")"
BUILD_DIR="$PROJECT_DIR/build_test"

DRIVER_DIR="$PROJECT_DIR/driver"

mkdir -p "$BUILD_DIR"

echo "Building sm_test..."
g++ -std=c++17 -Wall -Wextra -O2 \
    "$SCRIPT_DIR/sm_test.cpp" \
    -I"$DRIVER_DIR/include" \
    -I"$STATE_MACHINE_DIR" \
    -L"$BUILD_DIR/driver" \
    -ldog_driver \
    -lpthread \
    -o "$BUILD_DIR/sm_test"

echo "Done: $BUILD_DIR/sm_test"
