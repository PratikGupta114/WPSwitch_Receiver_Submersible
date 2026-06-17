#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

# Move to the project root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo "=================================================="
echo " Starting Host Unit Tests for Both Build Variants"
echo "=================================================="

# Color helpers
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 1. Compile & run HLW8032-enabled variant
echo -e "\n[1/2] Building and running HLW8032-enabled tests (FEATURE_HLW8032=1)..."
gcc -Iinclude -DFEATURE_HLW8032=1 test/test_comms_hlw.c src/comm_protocol.c src/hlw8032.c -o test_comms_hlw_enabled
./test_comms_hlw_enabled

# 2. Compile & run HLW8032-disabled variant
echo -e "\n[2/2] Building and running HLW8032-disabled tests (FEATURE_HLW8032=0)..."
# Note that we do not compile src/hlw8032.c or src/sw_uart.c here, proving they are completely optional.
gcc -Iinclude -DFEATURE_HLW8032=0 test/test_comms_hlw.c src/comm_protocol.c -o test_comms_hlw_disabled
./test_comms_hlw_disabled

# Clean up
rm -f test_comms_hlw_enabled test_comms_hlw_disabled

echo -e "\n${GREEN}=================================================="
echo " All variants compiled and ran successfully! 🎉"
echo "==================================================${NC}"
