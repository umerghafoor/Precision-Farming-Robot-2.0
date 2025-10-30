#!/bin/bash
# Run desktop application in Docker with display support

echo "Starting Desktop Application..."
cd "$(dirname "$0")"

# Enable X11 forwarding
xhost +local:docker

# Run the container
sudo docker run -it --rm \
    --name weed_detection_desktop \
    --network ros2_network \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=0 \
    -e ROS_LOCALHOST_ONLY=0 \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    weed-detection-desktop:latest

# Cleanup X11 permissions
xhost -local:docker

