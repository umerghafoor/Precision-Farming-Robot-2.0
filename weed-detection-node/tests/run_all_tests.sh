#!/bin/bash
# Run all Python unit tests in the tests directory

set -e

TEST_DIR="$(dirname "$0")"

echo "Running Python unit tests..."
python3 -m unittest discover "$TEST_DIR" 'test_*.py'

# For C++ tests, add commands here if/when available
# Example:
# cd ../../desktop-client/build && ./run_cpp_tests

echo "All tests completed."
