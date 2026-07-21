#!/bin/bash
# Unified build wrapper for Submersible ControlUnit accepting arbitrary board variant
# Usage: ./build.sh <variant_name> [idf.py arguments...]
# Example: ./build.sh rev2 build

if [ $# -eq 0 ]; then
    echo "Usage: $0 <variant_name> [idf.py arguments...]"
    echo "Example: $0 rev2 build"
    exit 1
fi

export BOARD_VARIANT="$1"
shift

# Source ESP-IDF environment if idf.py is not in PATH
if ! command -v idf.py &> /dev/null; then
    if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
        . "$HOME/esp/esp-idf/export.sh" > /dev/null 2>&1
    fi
fi

echo "--- Working on Submersible ControlUnit BOARD_VARIANT: ${BOARD_VARIANT} ---"

if [ $# -eq 0 ]; then
    idf.py -B "build_${BOARD_VARIANT}" build
else
    idf.py -B "build_${BOARD_VARIANT}" "$@"
fi
