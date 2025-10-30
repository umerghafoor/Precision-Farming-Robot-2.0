# Docker Guide - Weed Detection Node

## ✅ Docker Image Built Successfully!

Image: `weed-detection-node:latest`

## Quick Start - Docker Compose (Recommended)

### Option 1: Test with docker-compose (Easiest)

```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node

# Start all services
sudo docker-compose -f docker-compose.test.yml up

# To stop (Ctrl+C, then):
sudo docker-compose -f docker-compose.test.yml down
```

This will start:
- ArUco processor node
- Image publisher (automatically publishing test images)
- Both communicating via ROS2 topics

### Option 2: Run individual containers

#### Terminal 1 - ArUco Processor
```bash
sudo docker run -it --rm \
    --name aruco_processor \
    --network host \
    -e ROS_DOMAIN_ID=0 \
    weed-detection-node:latest \
    ros2 run weed_detection_node aruco_processor
```

#### Terminal 2 - Image Publisher
```bash
sudo docker run -it --rm \
    --name image_publisher \
    --network host \
    -e ROS_DOMAIN_ID=0 \
    weed-detection-node:latest \
    ros2 run weed_detection_node image_publisher /root/ros2_ws/src/weed_detection_node/test_scene_with_aruco.jpg 1.0
```

#### Terminal 3 - Check Coordinates
```bash
sudo docker run -it --rm \
    --network host \
    -e ROS_DOMAIN_ID=0 \
    weed-detection-node:latest \
    ros2 topic echo /coordinates
```

## Automated Testing

Run the comprehensive test:
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
sudo ./test_docker.sh
```

## Docker Commands Reference

### List running containers
```bash
sudo docker ps
```

### List all containers (including stopped)
```bash
sudo docker ps -a
```

### Stop a container
```bash
sudo docker stop <container_name>
```

### View logs
```bash
sudo docker logs <container_name>
```

### Access container shell
```bash
sudo docker exec -it <container_name> /bin/bash
```

### Remove all stopped containers
```bash
sudo docker container prune
```

## Docker Compose Commands

### Start all services
```bash
sudo docker-compose -f docker-compose.test.yml up
```

### Start in background (detached)
```bash
sudo docker-compose -f docker-compose.test.yml up -d
```

### View logs
```bash
sudo docker-compose -f docker-compose.test.yml logs -f
```

### Stop all services
```bash
sudo docker-compose -f docker-compose.test.yml down
```

### Restart a service
```bash
sudo docker-compose -f docker-compose.test.yml restart aruco_processor
```

## Networking

All containers use `--network host` which means:
- ✅ Containers can communicate with each other via ROS2
- ✅ Containers can communicate with host ROS2 nodes
- ✅ No port mapping needed
- ✅ Simple configuration

## Environment Variables

- `ROS_DOMAIN_ID=0` - Ensures all ROS2 nodes can discover each other

## Troubleshooting

### Container won't start
```bash
# Check logs
sudo docker logs <container_name>

# Check if image exists
sudo docker images | grep weed-detection-node
```

### ROS2 topics not visible
```bash
# Make sure ROS_DOMAIN_ID matches
# Check network mode is 'host'
```

### Permission denied
```bash
# Add your user to docker group
sudo usermod -aG docker $USER
newgrp docker

# Or use sudo
sudo docker ...
```

## Next Steps

✅ Docker image built  
✅ Nodes tested  
🔄 Next: Build PyQt6 Desktop Application (also in Docker)

The desktop application will:
- Run in a separate Docker container
- Connect to the ROS2 nodes via network
- Display camera feeds and coordinates
- Provide GUI controls

