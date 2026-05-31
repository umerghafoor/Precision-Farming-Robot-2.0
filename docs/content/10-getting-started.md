# Getting Started & Operations

End-to-end bring-up of the whole system, plus a consolidated troubleshooting table and developer how-tos.

---

## 1. Prerequisites

| Component | Requirement |
|-----------|-------------|
| Raspberry Pi | 4B or newer, Ubuntu 24.04 |
| ROS2 | Jazzy (Humble works with minor changes) |
| Motor MCU | Arduino Uno + Adafruit Motor Shield v1 |
| Sensor MCU | Arduino Nano (ATmega328P) |
| Firmware build | PlatformIO CLI |
| Desktop | Linux, Qt6 dev packages, CMake ≥ 3.16, C++17 |
| Python | 3.10+, `pyserial`, `paho-mqtt`, `numpy`, `Pillow`, `opencv-python`, `ultralytics`, `cv_bridge` |

---

## 2. Full system startup

### Step 1 — Flash the firmware

```bash
cd firmware            && pio run --target upload   # Arduino Uno (motor controller)
cd ../firmware_sensors && pio run --target upload   # Arduino Nano (sensor node)
```

Confirm each board prints its `NODE_ID:` banner over `pio device monitor` before moving on — the Pi nodes rely on it for auto-detection.

### Step 2 — Build & launch the ROS2 stack on the Pi

```bash
cd raspberry-pi/ros2_robot_ws
./setup.sh
source install/setup.sh
ros2 launch robot robot.launch.py     # webcam + motor bridge + imu + mqtt bridge
```

### Step 3 — (optional) YOLO detection

```bash
ros2 run yolo_detection yolo_detection_node \
  --ros-args -p model_path:=$PWD/../../best.pt -p confidence_threshold:=0.5
```

### Step 4 — (optional) MQTT broker for the mobile app

```bash
sudo systemctl start mosquitto
ros2 run mqtt_bridge mqtt_bridge_node --ros-args -p mqtt_host:=<broker_ip>
```

### Step 5 — Desktop client

```bash
cd desktop-client
./run.sh                # then ROS2 → Connect from the menu
```

### Step 6 — Verify

```bash
ros2 topic list
ros2 topic echo /robot_status
ros2 topic hz /camera/color_jpeg
ros2 topic echo /imu/data --once
mosquitto_sub -h <broker_ip> -t "robot/#" -v
```

---

## 3. Driving the robot

```bash
# forward 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0}}'

# rotate in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0}, angular: {z: 1.0}}'

# arc
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.5}}'

# stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0}, angular: {z: 0}}'

# servos & laser
ros2 topic pub /servo1/angle std_msgs/msg/Int16 "{data: 120}" -1
ros2 service call /laser/set std_srvs/srv/SetBool "{data: true}"
```

> If no `/cmd_vel` arrives within `cmd_timeout_sec` (0.5 s at the bridge, 1.0 s at the controller), velocity is zeroed automatically.

---

## 4. Consolidated troubleshooting

| Problem | Check |
|---------|-------|
| Motor not responding | Uno reachable? bridge logs its probe; pass `serial_port:=/dev/ttyUSB0`; 12 V to the Motor Shield |
| Wrong motor turns | channel mapping in `firmware/include/constants.h` |
| Brown-out resets (Uno) | motor inrush sagging the supply — separate the 12 V rail |
| IMU node keeps respawning | Nano not answering `WHOAMI`; check banner, SDA=A4/SCL=A5, `0x68` |
| IMU not found (Nano LED) | 2-pulse blink = not found; verify wiring + 3.3 V |
| Laser always on | flip `LASER_ACTIVE_LOW` in `firmware_sensors/include/constants.h` |
| No camera frames in YOLO | topic type must match; `/camera/color_jpeg` is `CompressedImage`; watch the 5 s frame watchdog |
| Desktop wrong colour | `bgr8` vs `rgb8` — auto-corrected for `bgr8`; feed it `/camera/color_jpeg` |
| Desktop build fails | install Qt6 dev packages; unset `ROS_DISTRO` for a stub build |
| MQTT not connecting | `ros2 topic echo /mqtt_bridge/status` → `connected`; verify host/port; install `paho-mqtt` |
| Images not in MQTT | install `numpy` + `Pillow`; raise `image_rate_hz` (default 0.5) |
| Nodes won't start | `echo $ROS_DISTRO`; `colcon build --symlink-install` |

---

## 5. Developer how-tos

### Add a ROS2 node
1. Create a package under `raspberry-pi/ros2_robot_ws/src/` (C++ with `CMakeLists.txt` + `package.xml`, or Python).
2. Implement the node; add it to `launch/robot.launch.py`.
3. Add MQTT bridging in `mqtt_bridge_node.py` if the mobile app needs it.

### Modify firmware
- Motor: `firmware/src/`, `firmware/include/`. Sensor: `firmware_sensors/src/`, `firmware_sensors/include/`.
- Rebuild & flash with `pio run --target upload`. Keep the `NODE_ID:` banner intact so auto-detection still works.

### Extend the desktop client
- See the [Desktop Client](#) page — add a `BaseWidget` subclass, register it in `WidgetManager`, wire it in `MainWindow`, and add CMake sources.

### Retrain the weed model
- Run `train_weed_detection.py` (or a notebook) against your dataset, then replace `yolo.pt` / `best.pt`. The YOLO node auto-resolves `yolo8n.pt` → `best.pt` → `yolo*.pt`, or set `YOLO_MODEL_PATH`.

---

## 6. Roadmap (from project notes)

- Replace placeholder GPIO in `motor_driver`/`encoder_node` with a real GPIO library (wiringPi / gpiozero / pigpio).
- TF2 frames, RViz visualisation, and Nav2 integration for autonomous navigation.
- Sensor fusion (Kalman/complementary filter) over IMU + odometry.
- Implement the mobile app (live video, joystick control, map, detection overlay).
- Obstacle detection / avoidance and a systemd auto-start service.

---

## 7. License & contact

License and contact details are noted as "to be added" in the root README.
