#!/bin/bash

# Start Docker container for Precision Farming Desktop Client
# This script runs the ros2-humble-qt6 container with GUI support

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where this script is located
# Use the project root so the container is run from the repository root
PROJECT_ROOT="."
if [ -d "$PROJECT_ROOT" ]; then
  SCRIPT_DIR="$(cd "$PROJECT_ROOT" && pwd)"
else
  echo -e "${RED}Error: project root '${PROJECT_ROOT}' not found!${NC}"
  exit 1
fi
echo "PROJECT_ROOT: $PROJECT_ROOT"
echo "SCRIPT_DIR: $SCRIPT_DIR"

# ROS2 network settings
# Keep this fixed so all ROS2 devices join the same communication domain.
ROS_DOMAIN_ID=0
HOST_CYCLONEDDS_CONFIG="${CYCLONEDDS_CONFIG_PATH:-$HOME/.ros/dds_config.xml}"
LAPTOP_IP="${LAPTOP_IP:-100.101.83.33}"
PI_IP="${PI_IP:-100.99.166.15}"

mkdir -p "$(dirname "$HOST_CYCLONEDDS_CONFIG")"
cat > "$HOST_CYCLONEDDS_CONFIG" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="${ROS_DOMAIN_ID}">
    <General>
      <NetworkInterfaceAddress>tailscale0</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="${PI_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

echo -e "${GREEN}=== Starting Docker Container ===${NC}"
echo -e "${YELLOW}Workspace: ${SCRIPT_DIR}${NC}"
echo -e "${YELLOW}ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}${NC}"
echo -e "${YELLOW}Laptop IP (CycloneDDS interface): ${LAPTOP_IP}${NC}"
echo -e "${YELLOW}Pi IP (CycloneDDS peer): ${PI_IP}${NC}"

# Check if Docker image exists
if ! sudo docker images | grep -q "ros2-humble-qt6"; then
    echo -e "${RED}Error: Docker image 'ros2-humble-qt6' not found!${NC}"
    echo -e "${YELLOW}Please build the image first:${NC}"
    echo -e "  ./build_docker.sh"
    exit 1
fi

# Allow X11 connections from local Docker containers
echo -e "${GREEN}Configuring X11 access...${NC}"
xhost +local:root > /dev/null 2>&1

DOCKER_ARGS=(
  -it --rm
  --network host
  -e DISPLAY="$DISPLAY"
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
  -v /tmp/.X11-unix:/tmp/.X11-unix
  -v "$HOME/.Xauthority:/root/.Xauthority"
  -v "/home/sani/c0d3/Precision-Farming-Robot-2.0/desktop-client:/workspace"
)

echo -e "${GREEN}Using CycloneDDS config: ${HOST_CYCLONEDDS_CONFIG}${NC}"
DOCKER_ARGS+=(
  -e CYCLONEDDS_URI=/root/.ros/dds_config.xml
  -v "$HOST_CYCLONEDDS_CONFIG:/root/.ros/dds_config.xml:ro"
)

# Run the Docker container
echo -e "${GREEN}Starting container...${NC}"
sudo docker run "${DOCKER_ARGS[@]}" \
  --name precision-farming-client \
  ros2-humble-qt6 \
  bash

# Cleanup X11 access after container exits
echo -e "${GREEN}Cleaning up X11 access...${NC}"
xhost -local:root > /dev/null 2>&1

echo -e "${GREEN}Container stopped.${NC}"

