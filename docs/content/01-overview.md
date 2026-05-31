# Precision Farming Robot 2.0 — Project Documentation

> An autonomous precision-agriculture robot combining embedded firmware, a ROS2 control stack, a computer-vision weed-detection pipeline, and remote monitoring via desktop and mobile clients.

This documentation is generated from a full read of the repository. Where the code and the original prose READMEs disagree, **the code is treated as the source of truth** and the discrepancy is called out explicitly (see [Known Discrepancies](#5-known-doc-vs-code-discrepancies)).

---

## 1. What the system does

The Precision Farming Robot 2.0 is a **4-wheel differential-drive ground robot** designed for autonomous field operations such as **weed detection and targeted treatment** (a laser actuator is wired for "shoot the weed" experiments). It is built as a set of independently deployable layers that talk over well-defined wire protocols (USB serial, SPI, ROS2 topics/services, and MQTT).

<!-- DIAGRAM:SYSTEM-ARCH:BEGIN -->
<div class="diagram">
<svg viewBox="0 0 760 400" role="img" aria-label="System architecture: robot, Raspberry Pi, and the three clients">
<defs><marker id="ar" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="var(--text-dim)"/></marker><marker id="ar-a" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="var(--accent)"/></marker><marker id="ar-b" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="var(--accent-2)"/></marker><marker id="ar-w" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="#d29922"/></marker></defs>
<rect class="group-box" x="20" y="20" width="430" height="360" rx="12"/><text class="t-grp" x="40" y="44">Field robot</text><rect class="node-accent" x="45" y="60" width="175" height="56" rx="9"/><text class="t" x="132.5" y="84" text-anchor="middle">Arduino Uno</text><text class="t-dim" x="132.5" y="100" text-anchor="middle">Motor Shield · 4 DC · 2 servo</text><rect class="node-accent" x="45" y="150" width="175" height="56" rx="9"/><text class="t" x="132.5" y="174" text-anchor="middle">Arduino Nano</text><text class="t-dim" x="132.5" y="190" text-anchor="middle">MPU IMU · laser (A0)</text><rect class="node" x="45" y="240" width="175" height="56" rx="9"/><text class="t" x="132.5" y="264" text-anchor="middle">Camera</text><text class="t-dim" x="132.5" y="280" text-anchor="middle">V4L2 / CSI</text><rect class="node-blue" x="270" y="130" width="160" height="110" rx="9"/><text class="t" x="350" y="173" text-anchor="middle">Raspberry Pi 4B</text><text class="t-dim" x="350" y="189" text-anchor="middle">Ubuntu 24.04</text><text class="t-dim" x="350" y="205" text-anchor="middle">ROS2 Jazzy</text><path class="edge" d="M220,88 L270,88 L270,150" marker-end="url(#ar)"/><rect class="elabel-bg" x="224" y="69" width="110" height="14"/><text class="elabel" x="226" y="80">USB serial 6-byte</text><path class="edge" d="M220,178 L268,178" marker-end="url(#ar)"/><rect class="elabel-bg" x="222" y="159" width="86" height="14"/><text class="elabel" x="224" y="170">A:/G: · LASER</text><path class="edge" d="M220,268 L270,268 L270,240" marker-end="url(#ar)"/><rect class="elabel-bg" x="224" y="275" width="44" height="14"/><text class="elabel" x="226" y="286">frames</text><rect class="node" x="560" y="40" width="180" height="56" rx="9"/><text class="t" x="650" y="64" text-anchor="middle">Mobile App</text><text class="t-dim" x="650" y="80" text-anchor="middle">MQTT telemetry</text><rect class="node" x="560" y="160" width="180" height="56" rx="9"/><text class="t" x="650" y="184" text-anchor="middle">Desktop Client</text><text class="t-dim" x="650" y="200" text-anchor="middle">Qt6 digital twin</text><rect class="node" x="560" y="280" width="180" height="56" rx="9"/><text class="t" x="650" y="304" text-anchor="middle">CV scripts</text><text class="t-dim" x="650" y="320" text-anchor="middle">YOLO train / infer</text><path class="edge-blue" d="M430,165 L495,165 L495,68 L556,68" marker-end="url(#ar-b)"/><rect class="elabel-bg" x="450" y="119" width="62" height="14"/><text class="elabel" x="452" y="130">MQTT 1883</text><path class="edge-blue" d="M430,185 L556,185" marker-end="url(#ar-b)"/><rect class="elabel-bg" x="450" y="167" width="56" height="14"/><text class="elabel" x="452" y="178">ROS2 DDS</text><path class="edge" d="M430,220 L495,220 L495,305 L556,305" marker-end="url(#ar)"/><rect class="elabel-bg" x="450" y="239" width="50" height="14"/><text class="elabel" x="452" y="250">weights</text>
</svg>
<div class="d-cap">System architecture — two Arduinos + camera feed the Raspberry Pi ROS2 stack, which serves the mobile app (MQTT), desktop client (ROS2 DDS), and offline CV tooling.</div>
</div>
<!-- DIAGRAM:SYSTEM-ARCH:END -->

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

<!-- DIAGRAM:DATA-FLOW:BEGIN -->
<div class="diagram">
<svg viewBox="0 0 855 340" role="img" aria-label="End-to-end data flow across three pipelines">
<defs><marker id="ar" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="var(--text-dim)"/></marker><marker id="ar-a" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="var(--accent)"/></marker><marker id="ar-b" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="var(--accent-2)"/></marker><marker id="ar-w" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="#d29922"/></marker></defs>
<rect class="node-accent" x="20" y="30" width="150" height="50" rx="9"/><text class="t" x="95" y="59" text-anchor="middle">webcam_node</text><rect class="node-blue" x="230" y="30" width="175" height="50" rx="9"/><text class="t" x="317.5" y="59" text-anchor="middle">yolo_detection</text><rect class="node-blue" x="465" y="30" width="160" height="50" rx="9"/><text class="t" x="545" y="59" text-anchor="middle">mqtt_bridge</text><rect class="node" x="685" y="30" width="150" height="50" rx="9"/><text class="t" x="760" y="59" text-anchor="middle">Mobile App</text><path class="edge-accent" d="M170,55 L226,55" marker-end="url(#ar-a)"/><rect class="elabel-bg" x="170" y="36" width="68" height="14"/><text class="elabel" x="172" y="47">color_jpeg</text><path class="edge-blue" d="M405,55 L461,55" marker-end="url(#ar-b)"/><rect class="elabel-bg" x="406" y="36" width="74" height="14"/><text class="elabel" x="408" y="47">results/img</text><path class="edge-blue" d="M625,55 L681,55" marker-end="url(#ar-b)"/><rect class="elabel-bg" x="628" y="36" width="50" height="14"/><text class="elabel" x="630" y="47">robot/*</text><rect class="node" x="20" y="150" width="150" height="50" rx="9"/><text class="t" x="95" y="179" text-anchor="middle">/cmd_vel</text><rect class="node-accent" x="230" y="150" width="175" height="50" rx="9"/><text class="t" x="317.5" y="179" text-anchor="middle">spi_controller_bridge</text><rect class="node" x="465" y="150" width="160" height="50" rx="9"/><text class="t" x="545" y="179" text-anchor="middle">Arduino Uno</text><rect class="node-accent" x="685" y="150" width="150" height="50" rx="9"/><text class="t" x="760" y="179" text-anchor="middle">4 DC motors</text><path class="edge" d="M170,175 L226,175" marker-end="url(#ar)"/><path class="edge-accent" d="M405,175 L461,175" marker-end="url(#ar-a)"/><rect class="elabel-bg" x="406" y="156" width="86" height="14"/><text class="elabel" x="408" y="167">6-byte serial</text><path class="edge-accent" d="M625,175 L681,175" marker-end="url(#ar-a)"/><rect class="elabel-bg" x="634" y="156" width="26" height="14"/><text class="elabel" x="636" y="167">PWM</text><rect class="node" x="20" y="270" width="150" height="50" rx="9"/><text class="t" x="95" y="299" text-anchor="middle">Arduino Nano</text><rect class="node-accent" x="230" y="270" width="175" height="50" rx="9"/><text class="t" x="317.5" y="299" text-anchor="middle">imu_node</text><rect class="node" x="465" y="270" width="160" height="50" rx="9"/><text class="t" x="545" y="299" text-anchor="middle">robot_controller</text><rect class="node" x="685" y="270" width="150" height="50" rx="9"/><text class="t-dim" x="760" y="299" text-anchor="middle">/odom · /imu/data</text><path class="edge" d="M170,295 L226,295" marker-end="url(#ar)"/><rect class="elabel-bg" x="174" y="276" width="38" height="14"/><text class="elabel" x="176" y="287">A:/G:</text><path class="edge" d="M405,295 L461,295" marker-end="url(#ar)"/><rect class="elabel-bg" x="406" y="276" width="62" height="14"/><text class="elabel" x="408" y="287">/imu/data</text><path class="edge" d="M625,295 L681,295" marker-end="url(#ar)"/><rect class="elabel-bg" x="626" y="276" width="86" height="14"/><text class="elabel" x="628" y="287">/cmd_vel_safe</text>
</svg>
<div class="d-cap">Three pipelines: vision (top), motion (middle), and sensing/safety (bottom). Green edges carry actuation/camera data; blue edges carry vision &amp; MQTT.</div>
</div>
<!-- DIAGRAM:DATA-FLOW:END -->

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
