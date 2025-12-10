#!/bin/bash

# Test runner script for Precision Farming Robot
# Runs all Python and C++ tests

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="/workspace/desktop-client"

cd /workspace/desktop-client/desktop-client/src/tests/build && rm -rf CMakeFiles CMakeCache.txt cmake_install.cmake Makefile 2>/dev/null; cmake .. >/dev/null 2>&1 && make 2>&1 | tail -15

echo "$PROJECT_ROOT"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Precision Farming Robot - Test Suite${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Run Python tests (weed-detection-node)
echo -e "${YELLOW}Running Python Unit Tests...${NC}"
echo -e "${YELLOW}=============================${NC}"

PYTHON_TEST_DIR="$PROJECT_ROOT/weed-detection-node/tests"
if [ -d "$PYTHON_TEST_DIR" ]; then
    cd "$PYTHON_TEST_DIR"
    python3 -m pytest -v test_*.py 2>/dev/null || python3 -m unittest discover -v -s "$PYTHON_TEST_DIR" -p 'test_*.py'
    PYTHON_RESULT=$?
else
    echo -e "${RED}Python test directory not found: $PYTHON_TEST_DIR${NC}"
    PYTHON_RESULT=1
fi

echo ""

# Run C++ tests (desktop-client)
echo -e "${YELLOW}Running C++ Unit Tests...${NC}"
echo -e "${YELLOW}===========================${NC}"

CPP_TEST_BUILD_DIR="$PROJECT_ROOT/desktop-client/src/tests/build"
CPP_TEST_EXECUTABLE="$CPP_TEST_BUILD_DIR/runDesktopClientTests"

if [ -f "$CPP_TEST_EXECUTABLE" ]; then
    echo "Executing: $CPP_TEST_EXECUTABLE"
    cd "$CPP_TEST_BUILD_DIR"
    "$CPP_TEST_EXECUTABLE" --gtest_color=yes
    CPP_RESULT=$?
else
    echo -e "${RED}C++ test executable not found: $CPP_TEST_EXECUTABLE${NC}"
    echo -e "${YELLOW}Please build C++ tests first with:${NC}"
    echo -e "  cd $CPP_TEST_BUILD_DIR"
    echo -e "  cmake .."
    echo -e "  make"
    CPP_RESULT=1
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Test Results Summary${NC}"
echo -e "${GREEN}========================================${NC}"

if [ $PYTHON_RESULT -eq 0 ]; then
    echo -e "${GREEN}✓ Python Tests: PASSED${NC}"
else
    echo -e "${RED}✗ Python Tests: FAILED${NC}"
fi

if [ $CPP_RESULT -eq 0 ]; then
    echo -e "${GREEN}✓ C++ Tests: PASSED${NC}"
else
    echo -e "${RED}✗ C++ Tests: FAILED${NC}"
fi

echo ""

if [ $PYTHON_RESULT -eq 0 ] && [ $CPP_RESULT -eq 0 ]; then
    echo -e "${GREEN}All tests passed!${NC}"
    exit 0
else
    echo -e "${RED}Some tests failed!${NC}"
    exit 1
fi
