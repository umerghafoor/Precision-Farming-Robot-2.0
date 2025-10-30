#!/bin/bash
# Test the Docker container setup

echo "======================================"
echo "Testing ROS2 Nodes in Docker"
echo "======================================"
echo ""

# Check if image exists
if ! sudo docker images | grep -q "weed-detection-node"; then
    echo "❌ Docker image 'weed-detection-node' not found!"
    echo "Please run: sudo docker build -t weed-detection-node:latest ."
    exit 1
fi

echo "✓ Docker image found"
echo ""

# Test 1: Run processor in background
echo "Test 1: Starting ArUco processor in Docker..."
sudo docker run -d --rm \
    --name test_processor \
    --network host \
    -e ROS_DOMAIN_ID=0 \
    weed-detection-node:latest \
    ros2 run weed_detection_node aruco_processor

sleep 3

# Check if it's running
if sudo docker ps | grep -q "test_processor"; then
    echo "✓ ArUco processor running"
else
    echo "❌ ArUco processor failed to start"
    exit 1
fi

# Test 2: Publish test image
echo ""
echo "Test 2: Publishing test image..."
sudo docker run -d --rm \
    --name test_publisher \
    --network host \
    -e ROS_DOMAIN_ID=0 \
    weed-detection-node:latest \
    ros2 run weed_detection_node image_publisher /root/ros2_ws/src/weed_detection_node/test_scene_with_aruco.jpg 1.0

sleep 5

# Test 3: Check topics
echo ""
echo "Test 3: Checking ROS2 topics..."
TOPICS=$(sudo docker run --rm \
    --network host \
    -e ROS_DOMAIN_ID=0 \
    weed-detection-node:latest \
    ros2 topic list)

echo "Active topics:"
echo "$TOPICS"
echo ""

if echo "$TOPICS" | grep -q "camera/raw"; then
    echo "✓ /camera/raw topic found"
else
    echo "❌ /camera/raw topic not found"
fi

if echo "$TOPICS" | grep -q "camera/annotated"; then
    echo "✓ /camera/annotated topic found"
else
    echo "❌ /camera/annotated topic not found"
fi

if echo "$TOPICS" | grep -q "coordinates"; then
    echo "✓ /coordinates topic found"
else
    echo "❌ /coordinates topic not found"
fi

# Test 4: Echo coordinates
echo ""
echo "Test 4: Reading coordinates (5 seconds)..."
timeout 5 sudo docker run --rm \
    --network host \
    -e ROS_DOMAIN_ID=0 \
    weed-detection-node:latest \
    ros2 topic echo /coordinates --once

# Cleanup
echo ""
echo "Cleaning up test containers..."
sudo docker stop test_processor test_publisher 2>/dev/null
sleep 2

echo ""
echo "======================================"
echo "✓ Docker test complete!"
echo "======================================"
echo ""
echo "To run the full system with docker-compose:"
echo "  sudo docker-compose -f docker-compose.test.yml up"
echo ""
echo "To stop:"
echo "  sudo docker-compose -f docker-compose.test.yml down"

