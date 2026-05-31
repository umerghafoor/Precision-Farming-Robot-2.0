# Precision Farming Robot 2.0 — Project Documentation

> An autonomous precision-agriculture robot combining embedded firmware, a ROS2 control stack, a computer-vision weed-detection pipeline, and remote monitoring via desktop and mobile clients.

This documentation is generated from a full read of the repository. Where the code and the original prose READMEs disagree, **the code is treated as the source of truth** and the discrepancy is called out explicitly (see [Known Discrepancies](#5-known-doc-vs-code-discrepancies)).

---

## 1. What the system does

The Precision Farming Robot 2.0 is a **4-wheel differential-drive ground robot** designed for autonomous field operations such as **weed detection and targeted treatment** (a laser actuator is wired for "shoot the weed" experiments). It is built as a set of independently deployable layers that talk over well-defined wire protocols (USB serial, SPI, ROS2 topics/services, and MQTT).

```
                ┌─────────────────────────────────────────────────────────────┐
                │                         FIELD ROBOT                          │
                │                                                               │
   12 V  ─────► │  ┌──────────────┐   USB serial 115200   ┌────────────────┐   │
   battery      │  │ Arduino Uno  │ ◄───── 6-byte ───────►│                │   │
                │  │ Motor Shield │   [Dir,Spd]×3          │                │   │
                │  │  4 DC motors │   + "S1:<deg>" servo   │                │   │
                │  │  2 servos    │                        │  Raspberry Pi  │   │
                │  └──────────────┘                        │   (Ubuntu 24,  │   │
                │  ┌──────────────┐   USB serial / SPI     │   ROS2 Jazzy)  │   │
                │  │ Arduino Nano │ ◄── IMU lines, laser ─►│                │   │
                │  │ MPU-9250 IMU │   "A:..,G:.." 100 Hz   │                │   │
                │  │  laser (D7)  │   "LASER_ON/OFF"        │                │   │
                │  └──────────────┘                        └───────┬────────┘   │
                │  ┌──────────────┐   V4L2 / CSI                   │            │
                │  │   Camera     │ ──────────────────────────────┘            │
                │  └──────────────┘                                            │
                └───────────────────────────────────┬──────────────────────────┘
                                                     │ MQTT (1883)  +  ROS2 DDS
                          ┌──────────────────────────┼───────────────────────────┐
                          ▼                          ▼                           ▼
                ┌──────────────────┐      ┌────────────────────┐      ┌────────────────────┐
                │   Mobile App     │      │   Desktop Client   │      │  Standalone CV     │
                │ (MQTT telemetry, │      │  (Qt6 digital twin,│      │  scripts (YOLO     │
                │  field control)  │      │  ROS2 live control)│      │  train / infer)    │
                └──────────────────┘      └────────────────────┘      └────────────────────┘
```

---

## 2. Layers at a glance

| Layer | Tech | Runs on | Role |
|-------|------|---------|------|
| **Motor firmware** (`firmware/`) | C++ / Arduino / PlatformIO | Arduino Uno + Adafruit Motor Shield v1 | Drives 4 DC motors + 2 servos from a 6-byte serial packet |
| **Sensor firmware** (`firmware_sensors/`) | C++ / Arduino / PlatformIO | Arduino Nano (ATmega328P) | Reads MPU IMU, drives laser, acts as SPI slave + serial sensor |
| **ROS2 control stack** (`raspberry-pi/`) | C++ & Python / ROS2 Jazzy | Raspberry Pi 4B / Ubuntu 24.04 | Camera, motor bridge, IMU, MQTT bridge, YOLO detection, odometry, safety controller |
| **Weed detection node** (`weed-detection-node/`, `ros2_ws/`) | Python / ROS2 | RPi or dev box | ArUco field-marker detection + YOLO weed detection nodes |
| **Desktop client** (`desktop-client/`) | C++17 / Qt6 (optional ROS2) | Linux workstation | Live telemetry, command control, 3D digital twin, dockable widgets |
| **Mobile app** (`mobile-app/`) | TBD (Flutter/RN/native) | Phone/tablet | Field-side MQTT telemetry & control (spec only, no code yet) |
| **CV training/inference** (root scripts + notebooks) | Python / Ultralytics YOLO | Dev box / Colab | Train and run the weed-detection model (`yolo.pt` / `best.pt`) |

---

## 3. Repository structure

```
Precision-Farming-Robot-2.0/
├── firmware/               # Arduino Uno — motor + servo controller (PlatformIO)
├── firmware_sensors/       # Arduino Nano — MPU IMU + laser node (PlatformIO)
│   └── tools/i2c_scanner/  # standalone I2C bus scanner sketch
├── raspberry-pi/
│   └── ros2_robot_ws/      # PRODUCTION ROS2 Jazzy workspace (see ROS2 stack page)
│       ├── src/
│       │   ├── camera_sensor/    # webcam_node, camera_node, video_loop_node (Python)
│       │   ├── motor_control/    # motor_driver + spi_controller_bridge (C++)
│       │   ├── imu_sensor/       # imu_node (C++, serial bridge to Nano)
│       │   ├── encoder_odometry/ # encoder_node (C++)
│       │   ├── robot_controller/ # robot_controller safety layer (C++)
│       │   ├── mqtt_bridge/      # mqtt_bridge_node (Python)
│       │   ├── yolo_detection/   # yolo_detection_node (Python)
│       │   └── robot/            # meta launch package (robot.launch.py)
│       ├── launch/robot.launch.py
│       └── config/robot_config.yaml
├── weed-detection-node/    # ROS2 ArUco/YOLO pipeline + test scenes (Python)
├── ros2_ws/                # secondary ROS2 ws (weed_detection package)
├── desktop-client/         # Qt6 C++ desktop app (digital twin, telemetry)
│   └── docs/               # the app's own design docs (ARCHITECTURE, DIAGRAMS, ...)
├── mobile-app/             # mobile control & monitoring app (spec)
├── test_nodes/             # test utilities (camera publisher + ArUco scenes)
├── docs/                   # ← THIS documentation site (index.html + content/*.md)
├── detect_weed.py          # standalone single-image weed detector (CLI)
├── infer.py                # batch / video YOLO inference helper
├── train_weed_detection.py # YOLO training script
├── visualize_gt.py         # ground-truth label visualiser
├── Weed_Detection_YOLO_FYP.ipynb / train_weed_detection.ipynb  # training notebooks
├── yolo.pt / best.pt / yolov8s.pt  # model weights
└── *.png / *.obj / *.mtl   # sample images, 3D robot models for the twin
```

---

## 4. End-to-end data flow

1. **Camera** (`webcam_node`) captures frames and publishes two compressed streams:
   `/camera/raw` (tiny gray4/gray8 packed) and `/camera/color_jpeg` (standard JPEG).
2. **YOLO detection** (`yolo_detection_node`) subscribes to the JPEG stream, runs inference, and publishes an annotated image to `/camera/detection` and structured results (JSON) to `/detections/results`.
3. **Motion commands** arrive on `/cmd_vel` (from the desktop client, the controller, or `ros2 topic pub`). The `spi_controller_bridge` converts them with differential-drive kinematics into a 6-byte packet sent over USB serial to the Arduino Uno, which actuates the motors. Servo angles ride the same serial link as `S1:<deg>` / `S2:<deg>` text commands.
4. **IMU**: the Arduino Nano streams `A:..,G:..` lines over USB serial; `imu_node` parses them, converts to SI units, and publishes `sensor_msgs/Imu` on `/imu/data`. It also exposes laser control via `/laser/set` (service) and `/laser/cmd` (topic).
5. **Odometry**: `encoder_node` integrates encoder counts into `nav_msgs/Odometry` on `/odom` (GPIO read is a placeholder/simulated in the current code).
6. **Safety**: `robot_controller` subscribes to `/cmd_vel`, clamps velocities, enforces command-timeout and low-battery emergency stop, and republishes to `/cmd_vel_safe`.
7. **MQTT bridge** (`mqtt_bridge_node`) re-publishes status, odometry, IMU, detections, and throttled camera frames to `robot/*` MQTT topics for the **mobile app**.
8. **Desktop client** connects over ROS2 DDS for live video, telemetry, command publishing, and a 3D digital twin; it can run fully standalone in simulation mode when ROS2 is unavailable.

---

## 5. Known doc-vs-code discrepancies

These are places where the older prose READMEs describe an *intended* design that differs from what the committed code actually does. The documentation pages here follow the code.

| Topic | README says | Code actually does |
|-------|-------------|--------------------|
| Laser pin (Nano) | `D7` | `LASER_PIN = A0` (active-LOW), see `firmware_sensors/include/constants.h` |
| IMU chip (Nano) | MPU-9250 (9-DOF, magnetometer) | Firmware reports **MPU-6050** and only accel+gyro are packed; magnetometer fields are dropped |
| SPI Nano→RPi packet | 18 bytes (9× int16, incl. mag) | `SPI_TX_LEN = 12` bytes (6× int16: ax,ay,az,gx,gy,gz) |
| RPi↔Nano transport in production | SPI master/slave | The shipped `imu_node` talks to the Nano over **USB serial** (parses `A:..,G:..`), not SPI |
| Camera topic payload | `sensor_msgs/Image` (raw) | `webcam_node` publishes `sensor_msgs/CompressedImage` (gray4/gray8 + jpeg) |
| Launch graph | motor_control + imu_sensor + encoder_odometry + robot_controller | `robot.launch.py` starts **webcam_node, spi_controller_bridge, imu_node, mqtt_bridge_node** |
| `robot_controller` output | `/cmd_vel_filtered` | publishes `/cmd_vel_safe`; the bridge in production listens directly on `/cmd_vel` |
| Desktop widgets | small fixed set | richer set incl. Detection Summary, Laser Calibration, IMU 3D, Robot 3D Model (see Desktop page) |

When in doubt, read the corresponding source file linked from each page.
