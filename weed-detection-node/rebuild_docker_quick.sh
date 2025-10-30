#!/bin/bash
# Quick rebuild - just run the docker build command

echo "Rebuilding Docker image..."
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
sudo docker build -t weed-detection-node:latest .

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Success! Now test with the 3 terminals"
else
    echo ""
    echo "✗ Build failed"
    exit 1
fi

