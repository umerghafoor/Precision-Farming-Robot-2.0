#!/bin/bash
# Terminal 3: Run Image Viewer in Docker (shows coordinates)

echo "================================================"
echo "Terminal 3: Coordinates Viewer (Docker)"
echo "================================================"
echo ""
echo "Listening to coordinates topic..."
echo "You will see JSON data with marker positions"
echo "Press Ctrl+C to stop"
echo ""

sudo docker run -it --rm \
    --name coordinates_viewer \
    --network ros2_network \
    -e ROS_DOMAIN_ID=0 \
    -e ROS_LOCALHOST_ONLY=0 \
    weed-detection-node:latest \
    bash -c "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 topic echo /coordinates"

