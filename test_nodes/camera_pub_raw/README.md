# Image Publisher for ROS2

This is a ROS2 node that publishes images from the `data` folder to the `camera/raw` topic for testing purposes.

## Prerequisites

- ROS2 (Humble, Iron, or later)
- Python 3
- OpenCV
- cv_bridge

## Installation

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Make sure ROS2 is sourced:
```bash
source /opt/ros/<your-ros-distro>/setup.bash
```

3. Install cv_bridge if not already installed:
```bash
sudo apt-get install ros-<your-ros-distro>-cv-bridge
```

## Setup

1. Place your test images in the `data` folder
   - Supported formats: JPG, JPEG, PNG, BMP, TIFF
   - Images will be published in alphabetical order
   - The node will automatically loop through all images

## Running the Node

### Basic Usage

```bash
python3 main.py
```

This will:
- Load all images from the `data` folder
- Publish them at 10 Hz to the `camera/raw` topic
- Loop continuously through the images
- Add frame counter overlay on each image

### With Custom Parameters

You can customize the publishing behavior using ROS2 parameters:

```bash
python3 main.py --ros-args \
  -p publish_rate:=30.0 \
  -p data_folder:=data \
  -p loop:=true \
  -p frames_per_image:=90 \
  -p resize_width:=1920 \
  -p resize_height:=1080
```

### Available Parameters

- `publish_rate` (float, default: 10.0): Publishing frequency in Hz
- `data_folder` (string, default: 'data'): Folder containing images (relative to script location)
- `loop` (bool, default: true): Loop through images continuously
- `frames_per_image` (int, default: 30): Number of frames to display each image before switching to the next
  - At 10 Hz, 30 frames = 3 seconds per image
  - At 30 Hz, 90 frames = 3 seconds per image
- `resize_width` (int, default: 0): Resize width (0 = keep original size)
- `resize_height` (int, default: 0): Resize height (0 = keep original size)

## Verifying the Topic

To check if images are being published:

```bash
# List all topics
ros2 topic list

# Check the publishing rate
ros2 topic hz camera/raw

# View topic info
ros2 topic info camera/raw

# Echo topic (not recommended for images)
ros2 topic echo camera/raw
```

## Viewing the Images

You can use `rqt_image_view` to visualize the published images:

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select the `camera/raw` topic from the dropdown.

## Example Usage Scenarios

### Default - 3 seconds per image at 10 Hz
```bash
python3 main.py
# Each image shown for 30 frames ÷ 10 Hz = 3 seconds
```

### High-frequency publishing with fast cycling
```bash
python3 main.py --ros-args -p publish_rate:=60.0 -p frames_per_image:=60
# Each image shown for 1 second (60 frames ÷ 60 Hz)
```

### Slow slideshow (5 seconds per image)
```bash
python3 main.py --ros-args -p publish_rate:=10.0 -p frames_per_image:=50
# Each image shown for 5 seconds (50 frames ÷ 10 Hz)
```

### Very slow slideshow (10 seconds per image)
```bash
python3 main.py --ros-args -p frames_per_image:=100
# Each image shown for 10 seconds (100 frames ÷ 10 Hz)
```

### Resize images to HD resolution
```bash
python3 main.py --ros-args -p resize_width:=1920 -p resize_height:=1080
```

### Publish once without looping
```bash
python3 main.py --ros-args -p loop:=false
```

### Use images from a different folder
```bash
python3 main.py --ros-args -p data_folder:=/path/to/my/images
```

## Image Overlay

Each published image includes an overlay showing:
- Current frame number
- Current image index (e.g., "Image: 1/5")

This helps verify the stream is working and images are cycling correctly.

## Troubleshooting

### No images found
Make sure your `data` folder contains image files:
```bash
ls -la data/
```

Supported formats: .jpg, .jpeg, .png, .bmp, .tiff, .tif

If you encounter import errors:
1. Make sure ROS2 is properly sourced
2. Install cv_bridge: `sudo apt-get install ros-<distro>-cv-bridge`
3. Install Python packages: `pip install -r requirements.txt`

If the topic doesn't appear:
1. Check that the node is running: `ros2 node list`
2. Verify the topic exists: `ros2 topic list`
3. Check for errors in the node output
