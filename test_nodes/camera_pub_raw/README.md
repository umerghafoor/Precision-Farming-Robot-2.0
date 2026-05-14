# `test_nodes/camera_pub_raw/` — Camera Test Publisher

A standalone ROS2 Python node that publishes static images from the `data/` folder to `camera/raw`. Used for offline testing of the vision pipeline (ArUco processor, weed detection, desktop client) without a physical camera.

---

## Included Test Images

| File | Contents |
|------|---------|
| `test_scene_1_aruco.jpg` | Field scene with ArUco marker(s) |
| `test_scene_2_aruco.jpg` | Field scene with ArUco marker(s) |
| `test_scene_with_aruco.jpg` | Combined test scene |

Place additional test images in `data/` in any supported format.

---

## Prerequisites

- ROS2 (Jazzy recommended; Humble/Iron also work)
- Python 3
- `opencv-python`
- `cv_bridge`

```bash
pip install -r requirements.txt
sudo apt install ros-<distro>-cv-bridge
source /opt/ros/<distro>/setup.bash
```

---

## Running

### Basic (default 10 Hz, 3 seconds per image)

```bash
python3 main.py
```

### Custom parameters

```bash
python3 main.py --ros-args \
  -p publish_rate:=30.0 \
  -p frames_per_image:=90 \
  -p data_folder:=data \
  -p loop:=true \
  -p resize_width:=640 \
  -p resize_height:=480
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_rate` | `10.0` Hz | Publishing frequency |
| `data_folder` | `data` | Image folder path (relative to script) |
| `loop` | `true` | Loop through images indefinitely |
| `frames_per_image` | `30` | Frames to hold each image before advancing |
| `resize_width` | `0` | Resize width (`0` = keep original) |
| `resize_height` | `0` | Resize height (`0` = keep original) |

**Duration per image** = `frames_per_image / publish_rate` seconds.

---

## Quick Reference

| Use case | Command |
|----------|---------|
| Default (3 s/image at 10 Hz) | `python3 main.py` |
| Fast cycle (1 s/image at 30 Hz) | `python3 main.py --ros-args -p publish_rate:=30.0 -p frames_per_image:=30` |
| Slow slideshow (10 s/image) | `python3 main.py --ros-args -p frames_per_image:=100` |
| Resize to 640×480 | `python3 main.py --ros-args -p resize_width:=640 -p resize_height:=480` |
| Publish once, no loop | `python3 main.py --ros-args -p loop:=false` |
| Custom image folder | `python3 main.py --ros-args -p data_folder:=/path/to/images` |

---

## Supported Image Formats

`.jpg`, `.jpeg`, `.png`, `.bmp`, `.tiff`, `.tif`

Images are published in alphabetical filename order. Each frame has a counter overlay for visual verification.

---

## Verifying the Stream

```bash
# Check topic exists
ros2 topic list | grep camera

# Verify publish rate
ros2 topic hz /camera/raw

# View in rqt
ros2 run rqt_image_view rqt_image_view
# Select camera/raw from dropdown
```

---

## Integration with Vision Pipeline

Run the publisher alongside the ArUco processor to test the full detection pipeline without hardware:

```bash
# Terminal 1 — publish test images
python3 test_nodes/camera_pub_raw/main.py

# Terminal 2 — run ArUco processor
cd weed-detection-node
python3 weed_detection_node/aruco_processor.py

# Terminal 3 — view annotated output
ros2 topic echo /image/coordinates
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `No images found` | Check `data/` folder exists and contains supported image files |
| Import error for `cv_bridge` | `sudo apt install ros-<distro>-cv-bridge` |
| Topic not appearing | Confirm ROS2 is sourced: `echo $ROS_DISTRO` |
| Node not in `ros2 node list` | Check for Python exception at startup in terminal output |
