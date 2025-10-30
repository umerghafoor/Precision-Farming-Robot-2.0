#!/bin/bash
# Create a shared Docker network for ROS2 nodes

echo "Creating Docker network for ROS2 communication..."

# Check if network exists
if sudo docker network ls | grep -q "ros2_network"; then
    echo "✓ Network 'ros2_network' already exists"
else
    sudo docker network create ros2_network
    echo "✓ Created network 'ros2_network'"
fi

echo ""
echo "Network ready! Now run the nodes."

