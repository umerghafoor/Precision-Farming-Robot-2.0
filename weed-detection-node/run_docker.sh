#!/bin/bash
# Run the weed detection node in Docker with display support

# Enable X11 forwarding for GUI (image viewer)
xhost +local:docker

# Run the container
docker run -it --rm \
    --name weed-detection-node \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/root/ros2_ws/src/weed_detection_node \
    weed-detection-node:latest \
    "$@"

# Cleanup X11 permissions
xhost -local:docker

