#!/bin/bash
# Full ROS2 system test - demonstrates all three topics working

echo "======================================"
echo "ROS2 Weed Detection Node - Full Test"
echo "======================================"
echo ""

# Source the workspace
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
source install/setup.bash

echo "This test will:"
echo "  1. Start the ArUco processor node"
echo "  2. Publish test images to camera/raw"
echo "  3. Show the topics being published"
echo ""
echo "You can then:"
echo "  - Run 'ros2 topic echo /coordinates' to see marker data"
echo "  - Run 'ros2 run weed_detection_node image_viewer camera/annotated' to see annotated images"
echo "  - Run 'ros2 run weed_detection_node image_viewer camera/raw' to see raw images"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo ""
echo "Starting in 3 seconds..."
sleep 3

# Start the processor node in background
echo ""
echo "[1/3] Starting ArUco processor node..."
ros2 run weed_detection_node aruco_processor &
PROCESSOR_PID=$!
sleep 2

# Start publishing images
echo "[2/3] Starting image publisher..."
ros2 run weed_detection_node image_publisher test_scene_with_aruco.jpg 0.5 &
PUBLISHER_PID=$!
sleep 3

# Show active topics
echo ""
echo "[3/3] Active ROS2 topics:"
echo "--------------------------------------"
ros2 topic list
echo "--------------------------------------"
echo ""

echo "✓ System is running!"
echo ""
echo "To view the results in separate terminals, run:"
echo "  Terminal 1: source install/setup.bash && ros2 topic echo /coordinates"
echo "  Terminal 2: source install/setup.bash && ros2 run weed_detection_node image_viewer camera/annotated"
echo ""
echo "Press Ctrl+C to stop..."

# Wait for user interrupt
trap "echo ''; echo 'Stopping nodes...'; kill $PROCESSOR_PID $PUBLISHER_PID 2>/dev/null; exit 0" INT
wait

