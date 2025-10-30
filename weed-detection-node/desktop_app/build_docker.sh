#!/bin/bash
# Build Docker image for desktop application

echo "Building Desktop App Docker image..."
cd "$(dirname "$0")"

sudo docker build -t weed-detection-desktop:latest .

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Desktop app Docker image built successfully!"
    echo ""
    echo "To run:"
    echo "  ./run_docker.sh"
else
    echo ""
    echo "✗ Build failed!"
    exit 1
fi

