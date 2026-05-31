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

<!-- DIAGRAM:ROS2-GRAPH:BEGIN -->
<div class="diagram">
<svg viewBox="0 0 870 430" role="img" aria-label="ROS2 node dataflow graph">
<defs><marker id="ar" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="var(--text-dim)"/></marker><marker id="ar-a" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="var(--accent)"/></marker><marker id="ar-b" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="var(--accent-2)"/></marker><marker id="ar-w" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="#d29922"/></marker></defs>
<rect class="node-accent" x="30" y="30" width="160" height="54" rx="9"/><text class="t" x="110" y="53" text-anchor="middle">webcam_node</text><text class="t-dim" x="110" y="69" text-anchor="middle">camera_sensor</text><rect class="node-blue" x="330" y="30" width="180" height="54" rx="9"/><text class="t" x="420" y="53" text-anchor="middle">yolo_detection_node</text><text class="t-dim" x="420" y="69" text-anchor="middle">yolo_detection</text><rect class="node-blue" x="660" y="130" width="180" height="54" rx="9"/><text class="t" x="750" y="153" text-anchor="middle">mqtt_bridge_node</text><text class="t-dim" x="750" y="169" text-anchor="middle">mqtt_bridge</text><rect class="node-warn" x="700" y="30" width="120" height="54" rx="9"/><text class="t" x="760" y="61" text-anchor="middle">MQTT broker</text><rect class="node-accent" x="30" y="190" width="190" height="54" rx="9"/><text class="t" x="125" y="213" text-anchor="middle">spi_controller_bridge</text><text class="t-dim" x="125" y="229" text-anchor="middle">motor_control</text><rect class="node" x="360" y="190" width="150" height="54" rx="9"/><text class="t" x="435" y="221" text-anchor="middle">Arduino Uno</text><rect class="node-accent" x="30" y="330" width="160" height="54" rx="9"/><text class="t" x="110" y="353" text-anchor="middle">imu_node</text><text class="t-dim" x="110" y="369" text-anchor="middle">imu_sensor</text><rect class="node" x="360" y="330" width="150" height="54" rx="9"/><text class="t" x="435" y="361" text-anchor="middle">Arduino Nano</text><path class="edge-accent" d="M190,50 L326,50" marker-end="url(#ar-a)"/><rect class="elabel-bg" x="194" y="31" width="116" height="14"/><text class="elabel" x="196" y="42">/camera/color_jpeg</text><path class="edge" d="M110,84 L110,130 L320,130" marker-end="url(#ar)"/><rect class="elabel-bg" x="128" y="113" width="122" height="14"/><text class="elabel" x="130" y="124">/camera/raw (gray4)</text><path class="edge-blue" d="M510,50 L600,50 L600,145 L656,145" marker-end="url(#ar-b)"/><rect class="elabel-bg" x="518" y="31" width="110" height="14"/><text class="elabel" x="520" y="42">/camera/detection</text><path class="edge-blue" d="M510,64 L580,64 L580,165 L656,165" marker-end="url(#ar-b)"/><rect class="elabel-bg" x="518" y="89" width="122" height="14"/><text class="elabel" x="520" y="100">/detections/results</text><text class="t-dim" x="30" y="180">/cmd_vel ►</text><path class="edge-accent" d="M220,216 L356,216" marker-end="url(#ar-a)"/><rect class="elabel-bg" x="242" y="197" width="86" height="14"/><text class="elabel" x="244" y="208">serial 6-byte</text><path class="edge" d="M360,356 L194,356" marker-end="url(#ar)"/><rect class="elabel-bg" x="226" y="337" width="80" height="14"/><text class="elabel" x="228" y="348">serial A:/G:</text><path class="edge" d="M190,340 L620,340 L620,188" marker-end="url(#ar)"/><rect class="elabel-bg" x="298" y="321" width="62" height="14"/><text class="elabel" x="300" y="332">/imu/data</text><path class="edge" d="M110,384 L110,410 L435,410 L435,386" marker-end="url(#ar)"/><rect class="elabel-bg" x="138" y="395" width="146" height="14"/><text class="elabel" x="140" y="406">/laser/set · /laser/cmd</text><path class="edge-blue" d="M750,130 L750,86" marker-end="url(#ar-b)"/><rect class="elabel-bg" x="756" y="101" width="32" height="14"/><text class="elabel" x="758" y="112">MQTT</text>
</svg>
<div class="d-cap">Production node graph from robot.launch.py plus the YOLO node. Green = motor/camera path, blue = vision/MQTT path.</div>
</div>
<!-- DIAGRAM:ROS2-GRAPH:END -->

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
