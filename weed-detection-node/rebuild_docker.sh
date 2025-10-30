#!/bin/bash
# Rebuild Docker image with fixes

echo "======================================"
echo "Rebuilding Docker Image with Fixes"
echo "======================================"
echo ""
echo "Fixes applied:"
echo "  ✓ NumPy version locked to <2.0 (cv_bridge compatibility)"
echo "  ✓ OpenCV API compatibility for multiple versions"
echo "  ✓ ROS2 version changed from Humble to Jazzy (matches desktop-client)"
echo ""

cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node

echo "Building Docker image..."
sudo docker build -t weed-detection-node:latest .

if [ $? -eq 0 ]; then
    echo ""
    echo "======================================"
    echo "✓ Docker image rebuilt successfully!"
    echo "======================================"
    echo ""
    echo "Now test again with:"
    echo "  Terminal 1: ./run_processor_docker.sh"
    echo "  Terminal 2: ./run_publisher_docker.sh"
    echo "  Terminal 3: ./run_viewer_docker.sh"
else
    echo ""
    echo "======================================"
    echo "✗ Docker build failed!"
    echo "======================================"
    exit 1
fi

