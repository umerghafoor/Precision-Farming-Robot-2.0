#!/bin/bash

# Build Docker image for Precision Farming Desktop Client
# This script builds the ros2-jazzy-qt6 image from the Dockerfile

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Building Docker Image for Desktop Client ===${NC}"
echo -e "${YELLOW}Image name: ros2-jazzy-qt6:latest${NC}"
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker is not installed!${NC}"
    echo "Please install Docker first: https://docs.docker.com/engine/install/"
    exit 1
fi

# Build the Docker image
echo -e "${GREEN}Building Docker image...${NC}"
sudo docker build -t ros2-jazzy-qt6:latest .

# Check if build was successful
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}=== Docker image built successfully! ===${NC}"
    echo ""
    echo "To run the container:"
    echo -e "  ${YELLOW}./start_docker.sh${NC}"
    echo ""
    echo "To verify the image:"
    echo -e "  ${YELLOW}sudo docker images | grep ros2-jazzy-qt6${NC}"
else
    echo -e "${RED}=== Docker build failed! ===${NC}"
    exit 1
fi
