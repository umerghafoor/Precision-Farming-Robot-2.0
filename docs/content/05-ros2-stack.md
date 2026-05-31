# ROS2 Control Stack (`raspberry-pi/ros2_robot_ws/`)

The production ROS2 **Jazzy** workspace running on the Raspberry Pi. It mixes C++ nodes (real-time bridges) and Python nodes (camera, vision, MQTT).

---

## 1. What `robot.launch.py` actually starts

The committed launch file (`launch/robot.launch.py`, package `robot`) brings up **four** nodes — this is the real boot graph, which differs from the older "4 control nodes" description:

| Order | Package | Executable | Purpose |
|-------|---------|-----------|---------|
| 1 | `camera_sensor` | `webcam_node` | Capture + publish camera (gray4 raw + jpeg) |
| 2 | `motor_control` | `spi_controller_bridge` | `/cmd_vel` → 6-byte serial to Arduino Uno + servos |
| 3 | `imu_sensor` | `imu_node` | Serial bridge to Arduino Nano; `/imu/data` + laser control (respawns) |
| 4 | `mqtt_bridge` | `mqtt_bridge_node` | ROS2 → MQTT for the mobile app |

`yolo_detection_node`, `encoder_node`, and `robot_controller` are **not** in this launch file — they're started separately when needed.

```
  webcam_node ──/camera/color_jpeg──► yolo_detection_node ──/camera/detection──┐
       │                                      │                                │
       └──/camera/raw (gray4)                 └──/detections/results──┐        │
                                                                      ▼        ▼
  /cmd_vel ──► spi_controller_bridge ──serial──► Arduino Uno     mqtt_bridge_node ──► MQTT
                                                                      ▲   ▲   ▲
  Arduino Nano ──serial──► imu_node ──/imu/data──────────────────────┘   │   │
                              │                                /odom──────┘   │
                              └──/laser/state, /laser/set      /robot_status──┘
```

---

## 2. Packages & nodes

### 2.1 `camera_sensor` (Python)

Three nodes; `webcam_node` is the one launched by default.

**`webcam_node`** — publishes **two** `sensor_msgs/CompressedImage` streams from the same captured frame:

- `/camera/raw` — gray4 (4-bit packed, two pixels per byte) or gray8, format string `gray4;WxH` / `gray8;WxH`. Minimal wire size for low-bandwidth links.
- `/camera/color_jpeg` — standard JPEG (`format = 'jpeg'`), displayable by `rqt_image_view`, `rviz2`, etc.

It auto-selects the first working `/dev/video*` device, sets MJPG capture, and logs per-frame timing/size.

| Parameter | Default | Notes |
|-----------|---------|-------|
| `device_index` | 0 | overridden by auto-detect |
| `image_width` / `image_height` | 1920 / 1080 | launch overrides to 640×480 |
| `publish_rate` | 10.0 Hz | |
| `frame_id` | `camera_link` | |
| `camera_topic` | `/camera/raw` | |
| `color_jpeg_topic` | `/camera/color_jpeg` | |
| `bit_depth` | 4 | 4 = gray4, else gray8 |
| `jpeg_quality` | 50 | |
| `artificial_delay` | 0.0 | inject latency for testing |

> Decoder helpers `decode_raw()` and `decode_color_jpeg()` are included at the bottom of `webcam_node.py` for copy-paste into any subscriber.

**`camera_node`** — Raspberry Pi CSI-camera variant.
**`video_loop_node`** — replays a video file as a looping JPEG `CompressedImage` stream (for testing without a camera; accepts a positional video path).

### 2.2 `motor_control` (C++)

**`spi_controller_bridge`** — the production motor path (despite the "SPI" name, it speaks **USB serial** to the Uno):

- Subscribes `/cmd_vel` (`geometry_msgs/Twist`), `/servo1/angle` & `/servo2/angle` (`std_msgs/Int16`).
- Auto-detects the motor controller by toggling DTR (reset) and reading the `NODE_ID:motor_controller` banner across `/dev/ttyUSB0-9` and `/dev/ttyACM0-9`.
- Differential-drive: `left = vx − vz·L/2`, `right = vx + vz·L/2`, normalised by `max_linear_velocity`, mapped to `(dir, 0–255)` per motor. Motor 3 mirrors the left side.
- Transmits the 6-byte packet on a timer (`tx_rate_hz`, default 20 Hz). Servo angles are sent as `S<id>:<deg>` text only on change.
- **Command timeout**: if no `/cmd_vel` within `cmd_timeout_sec` (default 0.5 s), velocity is zeroed.

| Parameter | Default |
|-----------|---------|
| `cmd_vel_topic` | `/cmd_vel` |
| `servo1_topic` / `servo2_topic` | `/servo1/angle` / `/servo2/angle` |
| `serial_port` | `auto` |
| `wheel_base` | 0.2 |
| `max_linear_velocity` | 1.0 |
| `cmd_timeout_sec` | 0.5 |
| `tx_rate_hz` | 20.0 |
| `default_servo_angle` | 90 |

**`motor_driver`** — the older placeholder L298N/GPIO node (kept for reference; GPIO calls are stubs).

### 2.3 `imu_sensor` (C++)

**`imu_node`** — serial bridge to the Arduino Nano:

- Auto-detects the sensor node: drains 1 s of IMU flood, sends `WHOAMI`, waits for `NODE_ID:sensor_node`.
- A background reader thread parses `A:ax,ay,az G:gx,gy,gz` lines via regex and converts to SI units:
  - accel: `raw / ACC_SCALE · G_TO_MS2` → m/s²
  - gyro: `raw / GYR_SCALE · DEG_TO_RAD` → rad/s
- Publishes `sensor_msgs/Imu` on `/imu/data` at `publish_rate_hz` (default 50 Hz). Orientation is marked unknown (`orientation_covariance[0] = -1`); accel/gyro covariances are fixed small constants.
- **Laser control**:
  - service `/laser/set` (`std_srvs/SetBool`)
  - topic `/laser/cmd` (`std_msgs/Bool`)
  - state echoed on `/laser/state` (`std_msgs/Bool`)
  - both write `LASER_ON\n` / `LASER_OFF\n` to the Nano.
- Launched with `respawn=True` (delay 5 s) so it recovers when the Nano is plugged in.

```bash
# turn the laser on/off
ros2 service call /laser/set std_srvs/srv/SetBool "{data: true}"
ros2 topic pub /laser/cmd std_msgs/msg/Bool "{data: false}" -1
```

### 2.4 `encoder_odometry` (C++) — placeholder

**`encoder_node`** publishes `nav_msgs/Odometry` on `/odom` at `update_rate` (20 Hz). Integrates left/right encoder deltas into x/y/θ with differential-drive kinematics and emits a quaternion. The GPIO read (`updateEncoderCounts`) is currently **simulated** (counts don't change) — wire in a real GPIO library to make it live.

| Parameter | Default |
|-----------|---------|
| `wheel_radius` | 0.05 |
| `wheel_base` | 0.2 |
| `counts_per_rev` | 20 |
| `update_rate` | 20.0 |

### 2.5 `robot_controller` (C++) — safety layer

Sits between `/cmd_vel` and the bridge as an optional safety wrapper:

- Subscribes `/cmd_vel`, `/imu/data`, `/odom`, `/battery_voltage`.
- Publishes safety-checked commands to **`/cmd_vel_safe`** and status strings to `/robot_status`.
- Clamps linear/angular velocity to limits; enforces command timeout; triggers **emergency stop** on low battery (`min_battery_voltage`, default 7.0 V) and keeps publishing zero Twist while stopped.

| Parameter | Default |
|-----------|---------|
| `control_loop_rate` | 20.0 Hz |
| `emergency_stop_enabled` | true |
| `max_linear_velocity` | 1.0 |
| `max_angular_velocity` | 2.0 |
| `min_battery_voltage` | 7.0 |
| `command_timeout` | 1.0 |
| `wheel_separation` | 0.2 |

> Note: in production the bridge listens directly on `/cmd_vel`. To insert the safety layer, point the bridge's `cmd_vel_topic` at `/cmd_vel_safe`.

### 2.6 `yolo_detection` (Python) — see the [Computer Vision](#) page for full detail.

### 2.7 `mqtt_bridge` (Python) — see the [MQTT & Mobile](#) page.

---

## 3. Configuration (`config/robot_config.yaml`)

A single YAML holds tunables for every subsystem (motor geometry, IMU rate, camera, encoder, controller safety, YOLO thresholds, GPIO pin map, topic names, logging, and MQTT host). Notable production values:

```yaml
imu_sensor:        { serial_port: auto, publish_rate_hz: 50.0 }
yolo_detection:    { confidence_threshold: 0.5, iou_threshold: 0.45, input_size: 640,
                     camera_topic: /camera/raw, annotated_topic: /detections/annotated,
                     results_topic: /detections/results }
mqtt_bridge:       { mqtt_host: sanilinux.mullet-bull.ts.net, mqtt_port: 1883,
                     mqtt_keepalive: 60, odom_rate_hz: 1.0 }
```

The MQTT host is a **Tailscale** hostname of the machine running Mosquitto. Override per-run with `--ros-args -p mqtt_host:=<ip>`.

---

## 4. Build & run

```bash
cd raspberry-pi/ros2_robot_ws

# build (one-time / after changes)
./setup.sh
# or: colcon build --symlink-install
# on a Pi, limit memory: colcon build --parallel-workers 1

source install/setup.sh

# launch the production graph
ros2 launch robot robot.launch.py

# camera helper script
./start_node.sh webcam            # USB webcam
./start_node.sh webcam /dev/video1
./start_node.sh camera            # Pi CSI camera

# drive it
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0}}'

# verify
ros2 topic list
ros2 topic hz /camera/color_jpeg
ros2 topic echo /robot_status
```

Build a single package: `colcon build --packages-select motor_control`.
Tune at runtime: `ros2 run imu_sensor imu_node --ros-args -p publish_rate_hz:=100.0`.

---

## 5. Troubleshooting

| Problem | Check |
|---------|-------|
| Motor controller not found | bridge logs which `/dev/tty*` it probed; pass `serial_port:=/dev/ttyUSB0` |
| IMU node keeps respawning | Nano not answering `WHOAMI`; check banner with `pio device monitor` |
| No camera frames in YOLO | confirm topic type matches (`/camera/color_jpeg` is `CompressedImage`); watch the 5 s frame watchdog warning |
| MQTT not connecting | `ros2 topic echo /mqtt_bridge/status` should read `connected`; verify host/port |
| Nodes won't start | `echo $ROS_DISTRO`; re-`colcon build --symlink-install` |
