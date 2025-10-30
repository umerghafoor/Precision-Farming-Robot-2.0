# Desktop Application - Complete Setup Guide

## 🎨 What You're Building

A beautiful PyQt6 desktop application that shows:
- **Real-time camera feeds** (raw and with detected markers)
- **Live coordinates** of detected ArUco markers
- **Start/Stop controls** for easy operation
- **Professional UI** with dark theme

---

## 📋 Step-by-Step Setup

### Step 1: Build the Desktop App Docker Image

```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node/desktop_app
./build_docker.sh
```

**Expected output:**
```
Building Desktop App Docker image...
✓ Desktop app Docker image built successfully!
```

**Time:** ~5-10 minutes (first time)

---

### Step 2: Start the ROS2 Backend Nodes

You need the processor and publisher running first.

**Terminal 1: ArUco Processor**
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_processor_docker.sh
```

**Wait 5 seconds**, then...

**Terminal 2: Image Publisher**
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_publisher_docker.sh
```

**Verify Terminal 1 shows:**
```
[INFO] [aruco_processor]: Received image: (720, 1280, 3)
[INFO] [aruco_processor]: Detected 5 ArUco markers ✓
```

---

### Step 3: Launch the Desktop Application

**Terminal 3: Desktop App**
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node/desktop_app
./run_docker.sh
```

**A window will open!** 🎉

---

## 🖱️ Using the Application

### When the Window Opens:

1. **Click the "🚀 Start" button**
   - Application connects to ROS2 topics
   - Status changes to "Running ✓"

2. **Watch the Magic Happen!**
   - **Top-left**: Raw camera image appears
   - **Bottom-left**: Annotated image with green boxes around markers
   - **Right side**: Coordinates of all 5 markers in real-time

3. **To Stop:**
   - Click "⏹ Stop" button
   - Or just close the window

---

## 🎯 What You'll See

```
╔════════════════════════════════════════════════════════╗
║  🚀 Start  ⏹ Stop      Status: Running ✓              ║
╠═══════════════════════════╦════════════════════════════╣
║ 📷 Raw Camera:            ║ Detected Markers           ║
║ ┌──────────────────────┐ ║ Markers: 5                 ║
║ │                      │ ║                            ║
║ │   [Test Scene]       │ ║ ═══ Marker ID 0 ═══        ║
║ │                      │ ║  Center: (274, 224)        ║
║ └──────────────────────┘ ║  Corners:                  ║
║                           ║    1. (200, 150)           ║
║ 🎯 Annotated:             ║    2. (349, 150)           ║
║ ┌──────────────────────┐ ║    ...                     ║
║ │    ╔═══╗             │ ║                            ║
║ │    ║ 0 ║   ╔═══╗     │ ║ ═══ Marker ID 1 ═══        ║
║ │    ╚═══╝   ║ 1 ║     │ ║  Center: (659, 259)        ║
║ │      ╔═══╗ ╚═══╝     │ ║  ...                       ║
║ │      ║ 2 ║  ╔═══╗    │ ║                            ║
║ │      ╚═══╝  ║ 3 ║    │ ║ (All 5 markers listed)     ║
║ │            ╔╩═══╝    │ ║                            ║
║ │            ║ 4 ║      │ ║                            ║
║ │            ╚═══╝      │ ║                            ║
║ └──────────────────────┘ ║                            ║
╠═══════════════════════════╩════════════════════════════╣
║ Status: Connected - Receiving data...                  ║
╚════════════════════════════════════════════════════════╝
```

---

## 🐳 Complete Docker Setup

If you want everything in Docker:

### Terminal 1: Processor
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_processor_docker.sh
```

### Terminal 2: Publisher
```bash
./run_publisher_docker.sh
```

### Terminal 3: Desktop App
```bash
cd desktop_app
./run_docker.sh
```

---

## 💻 Alternative: Run Locally (Without Docker)

If you prefer to run the desktop app outside Docker:

```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node/desktop_app
./run_local.sh
```

**Requirements:**
- PyQt6 installed (`pip install PyQt6`)
- ROS2 Jazzy sourced
- ROS2 nodes running (can be in Docker)

---

## 🔧 Troubleshooting

### Issue: "Can't connect to display"
**Solution:**
```bash
xhost +local:docker
```

### Issue: "No images showing"
**Solution:**
- Make sure Terminal 1 and 2 are running FIRST
- Wait 5 seconds between starting nodes
- Check: `ros2 topic list` should show `/camera/raw`, `/camera/annotated`, `/coordinates`

### Issue: Desktop app won't start
**Solution:**
- Check Docker image is built: `sudo docker images | grep weed-detection-desktop`
- Verify network exists: `sudo docker network ls | grep ros2_network`
- If needed: `cd .. && ./setup_network.sh`

### Issue: "Markers: 0" in the app
**Solution:**
- Check Terminal 1 shows "Detected 5 ArUco markers"
- Click Stop, then Start again in the desktop app
- Verify topics: `ros2 topic echo /coordinates --once`

---

## 📁 Full Project Structure

```
weed-detection-node/
├── weed_detection_node/          # ROS2 Python package
│   ├── aruco_processor.py
│   ├── image_publisher.py
│   └── image_viewer.py
├── desktop_app/                   # PyQt6 Desktop Application ← NEW!
│   ├── main.py
│   ├── gui/
│   │   ├── main_window.py
│   │   └── ros_thread.py
│   ├── Dockerfile
│   ├── build_docker.sh
│   ├── run_docker.sh
│   └── run_local.sh
├── run_processor_docker.sh
├── run_publisher_docker.sh
└── setup_network.sh
```

---

## ✅ Quick Test Checklist

- [ ] Desktop app Docker image built
- [ ] ROS2 network created (`ros2_network`)
- [ ] Terminal 1: Processor running and detecting markers
- [ ] Terminal 2: Publisher sending images
- [ ] Terminal 3: Desktop app opens
- [ ] Click "Start" in desktop app
- [ ] Camera feeds visible
- [ ] Coordinates showing "Markers: 5"

**All checked? You're ready to go! 🚀**

---

## 🎓 What You've Built

✅ **Complete ROS2 System**
- ArUco detection node
- Image processing pipeline
- Topic-based communication

✅ **Docker Deployment**
- Containerized nodes
- Networked communication
- Isolated environments

✅ **Professional Desktop GUI**
- Real-time visualization
- User-friendly interface
- Production-ready

**You now have a complete weed detection monitoring system!** 🌱🤖

