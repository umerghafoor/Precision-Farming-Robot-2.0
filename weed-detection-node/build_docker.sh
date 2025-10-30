#!/bin/bash
# Build Docker image for weed detection node

echo "Building Docker image for weed_detection_node..."
docker build -t weed-detection-node:latest .

if [ $? -eq 0 ]; then
    echo "✓ Docker image built successfully!"
    echo ""
    echo "To run the container:"
    echo "  docker run -it --rm weed-detection-node:latest"
    echo ""
    echo "To run a specific node:"
    echo "  docker run -it --rm weed-detection-node:latest ros2 run weed_detection_node aruco_processor"
else
    echo "✗ Docker build failed!"
    exit 1
fi

