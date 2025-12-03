#!/bin/bash

# Build script for Precision Farming Desktop Client
# Usage: ./build.sh [clean|debug|release]

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
BUILD_TYPE="Release"
CLEAN_BUILD=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        clean)
            CLEAN_BUILD=true
            shift
            ;;
        debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        release)
            BUILD_TYPE="Release"
            shift
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            echo "Usage: $0 [clean|debug|release]"
            exit 1
            ;;
    esac
done

echo -e "${GREEN}=== Precision Farming Desktop Client Build Script ===${NC}"
echo -e "Build type: ${YELLOW}${BUILD_TYPE}${NC}"

# Check if ROS2 is sourced (optional)
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS2 not sourced. Building without ROS2 support.${NC}"
    echo -e "${YELLOW}To enable ROS2: source /opt/ros/<distro>/setup.bash before building${NC}"
    USE_ROS2=OFF
else
    echo -e "${GREEN}Using ROS2 distribution: ${ROS_DISTRO}${NC}"
    USE_ROS2=ON
fi

# Clean build if requested
if [ "$CLEAN_BUILD" = true ]; then
    echo -e "${YELLOW}Cleaning previous build...${NC}"
    rm -rf build install log
fi

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo -e "${GREEN}Configuring with CMake...${NC}"
cmake .. \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DUSE_ROS2=${USE_ROS2}

# Build
echo -e "${GREEN}Building project...${NC}"
make -j$(nproc)

# Check if build was successful
if [ $? -eq 0 ]; then
    echo -e "${GREEN}=== Build completed successfully! ===${NC}"
    echo -e "Executable: ${YELLOW}$(pwd)/PrecisionFarmingDesktopClient${NC}"
    echo ""
    echo "To run the application:"
    echo -e "  ${YELLOW}./build/PrecisionFarmingDesktopClient${NC}"
else
    echo -e "${RED}=== Build failed! ===${NC}"
    exit 1
fi
