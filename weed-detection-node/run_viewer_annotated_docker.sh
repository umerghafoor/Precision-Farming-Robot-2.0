#!/bin/bash
# Terminal 3 Alternative: Run Image Viewer with GUI (shows annotated images)

echo "================================================"
echo "Terminal 3: Image Viewer (Docker with Display)"
echo "================================================"
echo ""
echo "This will show the annotated images with ArUco markers..."
echo "Press 'q' in the image window to quit"
echo ""

# Enable X11 forwarding
xhost +local:docker

sudo docker run -it --rm \
    --name image_viewer \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    weed-detection-node:latest \
    ros2 run weed_detection_node image_viewer camera/annotated

# Cleanup X11 permissions
xhost -local:docker

