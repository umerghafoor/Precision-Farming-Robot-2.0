#!/bin/bash
# Terminal 1: Run ArUco Processor in Docker

echo "================================================"
echo "Terminal 1: ArUco Processor (Docker)"
echo "================================================"
echo ""
echo "Starting ArUco processor node in Docker..."
echo "Press Ctrl+C to stop"
echo ""

sudo docker run -it --rm \
    --name aruco_processor \
    --network ros2_network \
    -e ROS_DOMAIN_ID=0 \
    -e ROS_LOCALHOST_ONLY=0 \
    weed-detection-node:latest \
    ros2 run weed_detection_node aruco_processor

