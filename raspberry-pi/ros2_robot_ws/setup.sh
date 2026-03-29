#!/bin/bash

# Build and Setup Script for ROS2 Robot Workspace
# Usage: ./setup.sh [--build] [--install-deps]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

echo "=========================================="
echo "ROS2 Robot Workspace Setup"
echo "=========================================="
echo "Workspace: $WS_DIR"

# Parse arguments
BUILD_ONLY=false
INSTALL_DEPS=false

while [[ $# -gt 0 ]]; do
  case $1 in
    --build)
      BUILD_ONLY=true
      shift
      ;;
    --install-deps)
      INSTALL_DEPS=true
      shift
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: ./setup.sh [--build] [--install-deps]"
      exit 1
      ;;
  esac
done

# Install dependencies if requested
if [ "$INSTALL_DEPS" = true ]; then
  echo ""
  echo "Installing ROS2 dependencies..."
  
    if command -v apt &> /dev/null; then
      # Check if ROS2 is installed in conda or system paths.
      if ! command -v ros2 &> /dev/null && [ ! -d "/opt/ros" ]; then
        echo "ROS2 not found in the active environment."
        echo "Activate your conda environment or install ROS2:"
        echo "  https://docs.ros.org/en/humble/Installation.html"
        exit 1
      fi
    
    # Install system dependencies
    sudo apt-get update
    sudo apt-get install -y build-essential
    
    # GPIO libraries (optional)
    echo "Installing GPIO libraries..."
    sudo apt-get install -y python3-pip python3-dev
    pip install --upgrade pip
    pip install gpiozero pigpio RPi.GPIO
    
    # I2C tools (optional)
    echo "Installing I2C tools..."
    sudo apt-get install -y i2c-tools libi2c-dev
    
    echo "Dependencies installed successfully!"
  else
    echo "apt not found. Please install dependencies manually."
    exit 1
  fi
fi

# Source ROS2 setup.
# Priority:
# 1) Active environment (e.g., conda) if ros2 is already on PATH
# 2) /opt/ros/<distro>/setup.sh fallback
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP_PATH="/opt/ros/${ROS_DISTRO_NAME}/setup.sh"

if command -v ros2 &> /dev/null; then
  ROS2_BIN_PATH="$(command -v ros2)"
  echo "Using ROS2 from current environment: ${ROS2_BIN_PATH}"
  if [ -n "${CONDA_PREFIX:-}" ]; then
    echo "Conda environment detected: ${CONDA_PREFIX}"
  fi
elif [ -f "$ROS_SETUP_PATH" ]; then
  source "$ROS_SETUP_PATH"
  echo "ROS2 ${ROS_DISTRO_NAME} sourced from ${ROS_SETUP_PATH}"
else
  echo "ERROR: ROS2 not found in PATH and ${ROS_SETUP_PATH} does not exist."
  echo "If using conda, activate your environment first."
  echo "Otherwise set ROS_DISTRO to your installed distro under /opt/ros."
  exit 1
fi

# Build workspace
echo ""
echo "Building ROS2 packages..."
cd "$WS_DIR"

if command -v colcon &> /dev/null; then
  # Use single worker on Raspberry Pi to avoid memory issues
  if [ "$BUILD_ONLY" = true ]; then
    colcon build --parallel-workers 1 --symlink-install
  else
    colcon build --parallel-workers 1
  fi
  
  if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Build successful!"
    echo ""
    echo "=========================================="
    echo "Setup Complete!"
    echo "=========================================="
    echo ""
    echo "To start using the workspace, run:"
    echo "  source $WS_DIR/install/setup.sh"
    echo ""
    echo "To launch all robot nodes, run:"
    echo "  ros2 launch robot robot.launch.py"
    echo ""
    echo "To run individual nodes, use:"
    echo "  ros2 run motor_control motor_driver"
    echo "  ros2 run imu_sensor imu_node"
    echo "  ros2 run encoder_odometry encoder_node"
    echo "  ros2 run robot_controller robot_controller"
    echo ""
  else
    echo "✗ Build failed!"
    exit 1
  fi
else
  echo "ERROR: colcon not found. Install colcon:"
  echo "  sudo apt install python3-colcon-common-extensions"
  exit 1
fi
