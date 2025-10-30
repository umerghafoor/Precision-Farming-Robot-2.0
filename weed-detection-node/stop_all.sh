#!/bin/bash
# Stop all running Docker containers for weed detection

echo "Stopping all weed detection containers..."

sudo docker stop aruco_processor 2>/dev/null
sudo docker stop image_publisher 2>/dev/null  
sudo docker stop coordinates_viewer 2>/dev/null
sudo docker stop image_viewer 2>/dev/null

echo "✓ All containers stopped"
echo ""
echo "You can now restart them in order:"
echo "  Terminal 1: ./run_processor_docker.sh"
echo "  Terminal 2: ./run_publisher_docker.sh (wait 3 seconds after Terminal 1)"
echo "  Terminal 3: ./run_viewer_docker.sh"

