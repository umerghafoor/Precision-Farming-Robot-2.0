# 🚀 Quick Start - Weed Detection Node in Docker

## **Exactly like local, but in Docker containers!**

---

## **Step-by-Step Instructions**

### **1. Open 3 Terminal Windows**

### **Terminal 1: Start ArUco Processor**
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_processor_docker.sh
```

**Wait until you see:**
```
[INFO] [aruco_processor]: ArUco Processor Node initialized
[INFO] [aruco_processor]: Subscribing to: camera/raw
[INFO] [aruco_processor]: Publishing to: camera/annotated, coordinates
```

✅ **Leave this terminal running!**

---

### **Terminal 2: Start Image Publisher**
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_publisher_docker.sh
```

**You should see:**
```
[INFO] [image_publisher]: Publishing to camera/raw at 1.0 Hz
[INFO] [image_publisher]: Published image to camera/raw
```

**And in Terminal 1, you'll see:**
```
[INFO] [aruco_processor]: Received image: (720, 1280, 3)
[INFO] [aruco_processor]: Detected 5 ArUco markers
[INFO] [aruco_processor]: Published annotated image
[INFO] [aruco_processor]: Published coordinates: 5 markers
```

✅ **Leave this terminal running!**

---

### **Terminal 3: View Results**

**Option A - See Coordinates (JSON data):**
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_viewer_docker.sh
```

**Option B - See Annotated Images (GUI window):**
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_viewer_annotated_docker.sh
```

✅ **This will show marker data or images in real-time!**

---

## **To Stop Everything**

Press **Ctrl+C** in each terminal window.

---

## **That's It!**

Everything works exactly like when you tested locally:
- ✅ Same 3 terminals
- ✅ Same topics: `camera/raw`, `camera/annotated`, `coordinates`
- ✅ Same ArUco detection (5 markers)
- ✅ But now **everything runs in Docker!**

---

## **Next: PyQt6 Desktop Application**

Once this is working, we'll create a PyQt6 desktop GUI that:
- Shows camera feeds in a window
- Displays coordinates in a nice panel
- Has start/stop buttons
- Also runs in Docker

**Ready to test? Open 3 terminals and follow the steps above!** 🚀

