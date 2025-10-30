#!/bin/bash
# Check ROS2 topics to diagnose communication issues

echo "======================================"
echo "Checking ROS2 Topics"
echo "======================================"
echo ""

echo "1. Checking from processor container..."
sudo docker exec aruco_processor bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list" 2>/dev/null

if [ $? -ne 0 ]; then
    echo "   ⚠ Processor container not running"
    echo ""
    echo "   Start it first: ./run_processor_docker.sh"
    exit 1
fi

echo ""
echo "2. Checking topic info for /camera/raw..."
sudo docker exec aruco_processor bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic info /camera/raw" 2>/dev/null

echo ""
echo "3. Checking topic info for /camera/annotated..."
sudo docker exec aruco_processor bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic info /camera/annotated" 2>/dev/null

echo ""
echo "4. Checking topic info for /coordinates..."
sudo docker exec aruco_processor bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic info /coordinates" 2>/dev/null

echo ""
echo "5. Checking nodes..."
sudo docker exec aruco_processor bash -c "source /opt/ros/jazzy/setup.bash && ros2 node list" 2>/dev/null

echo ""
echo "======================================"

