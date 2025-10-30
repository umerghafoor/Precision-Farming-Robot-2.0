# Troubleshooting - Docker ROS2 Communication

## Problem: Processor Not Receiving Images

You're seeing:
- ✅ Terminal 1: Processor initialized (but no "Received image" messages)
- ✅ Terminal 2: Publisher sending images
- ❌ Terminal 3: Viewer not showing coordinates

**This means the nodes can't communicate properly.**

---

## Solution: Restart in Correct Order

### Step 1: Stop Everything
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./stop_all.sh
```

Press **Ctrl+C** in all 3 terminals to stop them.

---

### Step 2: Start in Order (IMPORTANT!)

**Terminal 1: Start Processor FIRST**
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

**⏱️ WAIT 5 SECONDS** - Let ROS2 discovery complete

---

**Terminal 2: Start Publisher SECOND**
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_publisher_docker.sh
```

**You should immediately see in Terminal 1:**
```
[INFO] [aruco_processor]: Received image: (720, 1280, 3)
[INFO] [aruco_processor]: Detected 5 ArUco markers
[INFO] [aruco_processor]: Published annotated image
[INFO] [aruco_processor]: Published coordinates: 5 markers
```

**⏱️ WAIT 3 SECONDS** - Let messages flow

---

**Terminal 3: Start Viewer THIRD**
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_viewer_docker.sh
```

**You should see:**
```json
{
  "timestamp": 1761843447,
  "markers": [
    {"id": 4, "center": {"x": 854, "y": 554}, ...},
    ...
  ]
}
```

---

## If Still Not Working: Diagnostic Check

Run this to see what's happening:
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./check_topics.sh
```

This will show:
- What topics exist
- How many publishers/subscribers
- Which nodes are running

---

## Common Issues

### Issue 1: Containers Can't See Each Other
**Symptom:** No "Received image" messages in Terminal 1  
**Fix:** Make sure all use `--network host` (scripts already do this)

### Issue 2: ROS_DOMAIN_ID Mismatch
**Symptom:** Nodes run but don't communicate  
**Fix:** All containers use `ROS_DOMAIN_ID=0` (scripts already do this)

### Issue 3: Wrong Startup Order
**Symptom:** Publisher starts before processor subscribes  
**Fix:** Always start processor first, wait 5 seconds, then publisher

### Issue 4: Old Containers Running
**Symptom:** Conflicts with old containers  
**Fix:** Run `./stop_all.sh` first

---

## Expected Behavior

When working correctly:

**Terminal 1 (Processor):**
```
[INFO] [aruco_processor]: Received image: (720, 1280, 3)
[INFO] [aruco_processor]: Detected 5 ArUco markers
[INFO] [aruco_processor]: Published annotated image
[INFO] [aruco_processor]: Published coordinates: 5 markers
```
(Repeats every second)

**Terminal 2 (Publisher):**
```
[INFO] [image_publisher]: Published image to camera/raw
```
(Repeats every second)

**Terminal 3 (Viewer):**
```json
{
  "timestamp": ...,
  "markers": [ ... ]
}
```
(Repeats every second)

---

## Quick Test

Want to verify everything works? Run:
```bash
# Terminal 1
./run_processor_docker.sh

# Wait 5 seconds

# Terminal 2  
./run_publisher_docker.sh

# Wait 3 seconds

# Terminal 3
./run_viewer_docker.sh
```

All three should show activity!

