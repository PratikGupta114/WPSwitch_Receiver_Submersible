#!/bin/bash
# Host unit test execution wrapper for Submersible Control Unit
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Compiling Submersible Control Unit Host Test Suite ==="
g++ -std=c++20 -Wall -Wextra -O2 test_critical_protection_suite.cpp -o test_critical_protection_suite

echo "=== Executing Submersible Control Unit Host Test Suite ==="
./test_critical_protection_suite

rm -f test_critical_protection_suite
echo "=== Submersible Host Unit Tests Completed Successfully ==="
