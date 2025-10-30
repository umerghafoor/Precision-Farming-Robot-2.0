# Running in Docker - 3 Terminal Setup

Just like you did locally, but now everything runs in Docker containers!

## Setup (One Time)

Make scripts executable:
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
chmod +x run_*_docker.sh
```

## Run in 3 Separate Terminals

### Terminal 1: ArUco Processor
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_processor_docker.sh
```

**You should see:**
```
[INFO] [aruco_processor]: ArUco Processor Node initialized
[INFO] [aruco_processor]: Subscribing to: camera/raw
[INFO] [aruco_processor]: Publishing to: camera/annotated, coordinates
```

---

### Terminal 2: Image Publisher
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_publisher_docker.sh
```

**You should see:**
```
[INFO] [image_publisher]: Loaded image: test_scene_with_aruco.jpg
[INFO] [image_publisher]: Image shape: (720, 1280, 3)
[INFO] [image_publisher]: Publishing to camera/raw at 1.0 Hz
[INFO] [image_publisher]: Published image to camera/raw
```

---

### Terminal 3 (Option A): View Coordinates (Text)
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_viewer_docker.sh
```

**You should see:**
```json
{
  "timestamp": 1761839120,
  "markers": [
    {"id": 4, "center": {"x": 854, "y": 554}, ...},
    {"id": 3, "center": {"x": 364, "y": 514}, ...},
    ...
  ]
}
```

### Terminal 3 (Option B): View Annotated Images (GUI)
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_viewer_annotated_docker.sh
```

**You should see:** A window with the image showing green boxes around ArUco markers!

---

## Checking What's Running

### See all Docker containers:
```bash
sudo docker ps
```

You should see:
- `aruco_processor`
- `image_publisher`
- `coordinates_viewer` (or `image_viewer`)

---

## Stopping Everything

Press **Ctrl+C** in each terminal window, or:

```bash
sudo docker stop aruco_processor image_publisher coordinates_viewer image_viewer
```

---

## Checking Topics from Another Container

You can also check topics from your existing desktop-client container:

```bash
sudo docker exec -it precision-farming-client bash
source /opt/ros/jazzy/setup.sh
ros2 topic list
```

You should see:
- `/camera/raw`
- `/camera/annotated`
- `/coordinates`

---

## Same as Local, But in Docker! ✅

Everything works exactly the same:
1. **Terminal 1** → Processor listens to camera/raw
2. **Terminal 2** → Publisher sends images to camera/raw
3. **Terminal 3** → Viewer shows coordinates or annotated images

The only difference: Everything runs in isolated Docker containers!

