#!/bin/bash
# Quick automated ROS2 test - verifies the system is working

cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
source install/setup.bash

echo "======================================"
echo "ROS2 Quick Test"
echo "======================================"
echo ""

# Start processor
echo "Starting ArUco processor..."
ros2 run weed_detection_node aruco_processor > /tmp/processor.log 2>&1 &
PROCESSOR_PID=$!
sleep 2

# Publish one image
echo "Publishing test image..."
timeout 3 ros2 run weed_detection_node image_publisher test_scene_with_aruco.jpg 1.0 > /tmp/publisher.log 2>&1 &
PUBLISHER_PID=$!
sleep 4

# Check topics
echo ""
echo "Active topics:"
ros2 topic list

echo ""
echo "Checking if data is being published..."

# Check camera/raw topic
if ros2 topic info /camera/raw >/dev/null 2>&1; then
    echo "  ✓ /camera/raw topic exists"
else
    echo "  ✗ /camera/raw topic NOT found"
fi

# Check camera/annotated topic
if ros2 topic info /camera/annotated >/dev/null 2>&1; then
    echo "  ✓ /camera/annotated topic exists"
else
    echo "  ✗ /camera/annotated topic NOT found"
fi

# Check coordinates topic
if ros2 topic info /coordinates >/dev/null 2>&1; then
    echo "  ✓ /coordinates topic exists"
else
    echo "  ✗ /coordinates topic NOT found"
fi

echo ""
echo "Processor log (last 10 lines):"
echo "--------------------------------------"
tail -n 10 /tmp/processor.log
echo "--------------------------------------"

# Cleanup
echo ""
echo "Cleaning up..."
kill $PROCESSOR_PID $PUBLISHER_PID 2>/dev/null
sleep 1

echo ""
echo "✓ Test complete!"
echo ""
echo "To run the full interactive system, use:"
echo "  ./test_ros2_full.sh"

