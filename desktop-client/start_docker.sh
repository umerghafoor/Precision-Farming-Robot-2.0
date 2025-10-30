#!/bin/bash

# Start Docker container for Precision Farming Desktop Client
# This script runs the ros2-jazzy-qt6 container with GUI support

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${GREEN}=== Starting Docker Container ===${NC}"
echo -e "${YELLOW}Workspace: ${SCRIPT_DIR}${NC}"

# Check if Docker image exists
if ! sudo docker images | grep -q "ros2-jazzy-qt6"; then
    echo -e "${RED}Error: Docker image 'ros2-jazzy-qt6' not found!${NC}"
    echo -e "${YELLOW}Please build the image first:${NC}"
    echo -e "  ./build_docker.sh"
    exit 1
fi

# Allow X11 connections from local Docker containers
echo -e "${GREEN}Configuring X11 access...${NC}"
xhost +local:root > /dev/null 2>&1

# Run the Docker container
echo -e "${GREEN}Starting container...${NC}"
sudo docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${SCRIPT_DIR}":/workspace/desktop-client \
  --name precision-farming-client \
  ros2-jazzy-qt6

# Cleanup X11 access after container exits
echo -e "${GREEN}Cleaning up X11 access...${NC}"
xhost -local:root > /dev/null 2>&1

echo -e "${GREEN}Container stopped.${NC}"

