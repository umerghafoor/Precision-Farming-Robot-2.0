#!/bin/bash
# Run desktop application locally (not in Docker)

echo "Starting Desktop Application (Local)..."
cd "$(dirname "$0")"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠ ROS2 not sourced. Sourcing..."
    source /opt/ros/jazzy/setup.bash
fi

# Check if weed_detection_node is sourced
if [ -d "../install" ]; then
    echo "⚠ Sourcing weed_detection_node workspace..."
    source ../install/setup.bash
fi

# Install requirements if needed
if ! python3 -c "import PyQt6" 2>/dev/null; then
    echo "⚠ Installing PyQt6..."
    pip3 install -r requirements.txt
fi

# Set environment
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Run the application
python3 main.py

