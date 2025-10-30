#!/bin/bash
# Quick test script for the weed detection node

echo "======================================"
echo "Weed Detection Node - System Test"
echo "======================================"
echo ""

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built. Building now..."
    colcon build
    if [ $? -ne 0 ]; then
        echo "❌ Build failed!"
        exit 1
    fi
    echo "✓ Build successful!"
fi

# Source the workspace
source install/setup.bash

echo ""
echo "Testing ROS2 package..."
echo ""

# Check if package is available
if ros2 pkg list | grep -q "weed_detection_node"; then
    echo "✓ Package 'weed_detection_node' found"
else
    echo "❌ Package 'weed_detection_node' not found"
    exit 1
fi

# Check executables
echo ""
echo "Checking executables..."
if ros2 pkg executables weed_detection_node | grep -q "aruco_processor"; then
    echo "✓ aruco_processor executable found"
else
    echo "❌ aruco_processor executable not found"
fi

if ros2 pkg executables weed_detection_node | grep -q "image_publisher"; then
    echo "✓ image_publisher executable found"
else
    echo "❌ image_publisher executable not found"
fi

if ros2 pkg executables weed_detection_node | grep -q "image_viewer"; then
    echo "✓ image_viewer executable found"
else
    echo "❌ image_viewer executable not found"
fi

# Check test images
echo ""
echo "Checking test images..."
if [ -f "test_scene_with_aruco.jpg" ]; then
    echo "✓ Test image found: test_scene_with_aruco.jpg"
else
    echo "⚠ Test image not found. Generating..."
    python3 generate_test_markers.py
fi

echo ""
echo "======================================"
echo "✓ System check complete!"
echo "======================================"
echo ""
echo "To run the system:"
echo "  Terminal 1: ros2 run weed_detection_node aruco_processor"
echo "  Terminal 2: ros2 run weed_detection_node image_publisher test_scene_with_aruco.jpg"
echo "  Terminal 3: ros2 run weed_detection_node image_viewer camera/annotated"
echo ""
echo "Or see QUICKSTART.md for detailed instructions."

