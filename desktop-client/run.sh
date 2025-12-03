#!/bin/bash

# Precision Farming Desktop Client - Launcher Script
# This script runs the application with proper output handling

cd "$(dirname "$0")"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Precision Farming Desktop Client ===${NC}"
echo ""

# Check if executable exists
if [ ! -f "build/PrecisionFarmingDesktopClient" ]; then
    echo -e "${YELLOW}Executable not found. Building...${NC}"
    ./build.sh release
    if [ $? -ne 0 ]; then
        echo "Build failed!"
        exit 1
    fi
fi

# Check if ROS2 is sourced
if [ -n "$ROS_DISTRO" ]; then
    echo -e "${GREEN}✓ ROS2 detected: $ROS_DISTRO${NC}"
else
    echo -e "${YELLOW}⚠ ROS2 not detected - Running in standalone mode${NC}"
fi

echo ""
echo -e "${GREEN}Starting application...${NC}"
echo "Log file: PrecisionFarmingClient.log"
echo ""

# Run the application
./build/PrecisionFarmingDesktopClient "$@"

EXIT_CODE=$?
echo ""
echo -e "${BLUE}Application exited with code: $EXIT_CODE${NC}"

# Show last few log entries
if [ -f "PrecisionFarmingClient.log" ]; then
    echo ""
    echo -e "${BLUE}Last log entries:${NC}"
    tail -n 10 PrecisionFarmingClient.log
fi

exit $EXIT_CODE
