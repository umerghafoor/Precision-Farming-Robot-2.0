# Weed Detection Node - ROS2 ArUco Processing

A ROS2 node for processing images with ArUco marker detection. This node is part of the Precision Farming Robot project and will later be extended for weed detection.

## Overview

This package contains:
- **ArUco Processor Node**: Subscribes to raw images, detects ArUco markers, and publishes annotated images and coordinates
- **Image Publisher**: Helper node to publish images from files to test the system
- **Image Viewer**: Helper node to view processed images and coordinates

## Topics

### Subscribed Topics
- `camera/raw` (sensor_msgs/Image): Raw camera images

### Published Topics
- `camera/annotated` (sensor_msgs/Image): Processed images with ArUco markers highlighted
- `coordinates` (std_msgs/String): JSON data containing detected marker IDs and coordinates

## Installation

### Prerequisites
- ROS2 (Humble or later recommended)
- Python 3.8+
- OpenCV with ArUco support

### Install Python Dependencies
```bash
pip install -r requirements.txt
```

### Build the Package
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
colcon build
source install/setup.bash
```

## Usage

### Terminal 1: Run the ArUco Processor Node
```bash
source install/setup.bash
ros2 run weed_detection_node aruco_processor
```

### Terminal 2: Publish an Image
```bash
source install/setup.bash
ros2 run weed_detection_node image_publisher <path_to_image.jpg> [rate_hz]
```

Example:
```bash
ros2 run weed_detection_node image_publisher test_image.jpg 1.0
```

### Terminal 3: View the Annotated Image
```bash
source install/setup.bash
ros2 run weed_detection_node image_viewer camera/annotated
```

Or view the raw image:
```bash
ros2 run weed_detection_node image_viewer camera/raw
```

## Alternative: Publish Images Directly via Command Line

You can also publish images directly using `ros2 topic pub`, but it's more complex. The image_publisher node is recommended.

## Testing

1. Generate test ArUco markers (see `generate_test_markers.py`)
2. Run the processor node
3. Publish a test image with ArUco markers
4. View the annotated output

## Coordinate Format

The coordinates topic publishes JSON data in the following format:
```json
{
  "timestamp": 1234567890,
  "markers": [
    {
      "id": 0,
      "center": {
        "x": 320,
        "y": 240
      },
      "corners": [
        {"x": 300, "y": 220},
        {"x": 340, "y": 220},
        {"x": 340, "y": 260},
        {"x": 300, "y": 260}
      ]
    }
  ]
}
```

## Future Development

This node currently detects ArUco markers for testing. Future versions will:
- Detect weeds in camera images
- Provide weed coordinates for robot navigation
- Integrate with the desktop client and PyQt6 application

## License

MIT

