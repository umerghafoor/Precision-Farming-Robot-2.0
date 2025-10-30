# Quick Start Guide - Weed Detection Node

## Step-by-Step Setup and Testing

### Step 1: Build the ROS2 Package

```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
colcon build
source install/setup.bash
```

### Step 2: Test the System

#### Terminal 1 - Start the ArUco Processor
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
source install/setup.bash
ros2 run weed_detection_node aruco_processor
```

You should see:
```
[INFO] [aruco_processor]: ArUco Processor Node initialized
[INFO] [aruco_processor]: Subscribing to: camera/raw
[INFO] [aruco_processor]: Publishing to: camera/annotated, coordinates
```

#### Terminal 2 - Publish a Test Image
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
source install/setup.bash
ros2 run weed_detection_node image_publisher test_scene_with_aruco.jpg 1.0
```

You should see the processor detect markers in Terminal 1!

#### Terminal 3 - View the Annotated Image
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
source install/setup.bash
ros2 run weed_detection_node image_viewer camera/annotated
```

A window will open showing the image with detected ArUco markers highlighted!

### Step 3: Check the Topics

In another terminal, you can check active topics:
```bash
ros2 topic list
```

You should see:
- `/camera/raw`
- `/camera/annotated`
- `/coordinates`

### Step 4: Subscribe to Coordinates

To see the detected marker coordinates:
```bash
ros2 topic echo /coordinates
```

You'll see JSON output with marker IDs and their positions!

## Using Your Own Images

1. Take or download an image with ArUco markers
2. Place it in the `weed-detection-node` folder
3. Run the publisher with your image:
   ```bash
   ros2 run weed_detection_node image_publisher your_image.jpg
   ```

## Docker Usage (Optional)

### Build Docker Image
```bash
./build_docker.sh
```

### Run in Docker
```bash
./run_docker.sh ros2 run weed_detection_node aruco_processor
```

## Troubleshooting

### Issue: "Package not found"
**Solution:** Make sure you sourced the workspace:
```bash
source install/setup.bash
```

### Issue: "No module named 'cv2'"
**Solution:** Install OpenCV:
```bash
pip install opencv-python opencv-contrib-python
```

### Issue: "Image viewer window not showing"
**Solution:** Check your DISPLAY variable and X11 forwarding:
```bash
echo $DISPLAY
```

## Next Steps

This node currently detects ArUco markers. To adapt it for weed detection:
1. Replace the ArUco detection code with your weed detection model
2. Update the coordinate output format for weed positions
3. Integrate with your robot control system

The basic ROS2 structure (topics, publishers, subscribers) will remain the same!

