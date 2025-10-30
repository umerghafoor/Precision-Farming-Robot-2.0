#!/bin/bash
# Terminal 2: Run Image Publisher in Docker

echo "================================================"
echo "Terminal 2: Image Publisher (Docker)"
echo "================================================"
echo ""
echo "Publishing test images to camera/raw topic..."
echo "Press Ctrl+C to stop"
echo ""

sudo docker run -it --rm \
    --name image_publisher \
    --network ros2_network \
    -e ROS_DOMAIN_ID=0 \
    -e ROS_LOCALHOST_ONLY=0 \
    weed-detection-node:latest \
    ros2 run weed_detection_node image_publisher /root/ros2_ws/src/weed_detection_node/test_scene_with_aruco.jpg 1.0

