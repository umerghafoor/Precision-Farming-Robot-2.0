#!/bin/bash
# Terminal 3 (Better): Pretty Coordinates Viewer

echo "================================================"
echo "Terminal 3: Pretty Coordinates Viewer (Docker)"
echo "================================================"
echo ""
echo "Displaying marker coordinates in readable format..."
echo "Press Ctrl+C to stop"
echo ""

sudo docker run -it --rm \
    --name pretty_viewer \
    --network ros2_network \
    -e ROS_DOMAIN_ID=0 \
    -e ROS_LOCALHOST_ONLY=0 \
    weed-detection-node:latest \
    ros2 run weed_detection_node pretty_viewer

