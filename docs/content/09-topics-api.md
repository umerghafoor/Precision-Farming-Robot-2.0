# Topic, Service & Wire-Protocol Reference

A consolidated reference for every interface in the system: ROS2 topics & services, MQTT topics, and the low-level serial/SPI packets.

---

## 1. ROS2 topics

| Topic | Type | Publisher(s) | Subscriber(s) |
|-------|------|--------------|---------------|
| `/cmd_vel` | `geometry_msgs/Twist` | desktop client, `robot_controller` (via `/cmd_vel_safe`), `ros2 topic pub` | `spi_controller_bridge`, `robot_controller` |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | `robot_controller` | `spi_controller_bridge` (when configured) |
| `/servo1/angle`, `/servo2/angle` | `std_msgs/Int16` | desktop / scripts | `spi_controller_bridge` |
| `/imu/data` | `sensor_msgs/Imu` | `imu_node` | `robot_controller`, `mqtt_bridge`, desktop client |
| `/laser/cmd` | `std_msgs/Bool` | scripts / desktop | `imu_node` |
| `/laser/state` | `std_msgs/Bool` | `imu_node` | monitors |
| `/odom` | `nav_msgs/Odometry` | `encoder_node` | `robot_controller`, `mqtt_bridge` |
| `/battery_voltage` | `std_msgs/Float32` | (external) | `robot_controller` |
| `/robot_status` | `std_msgs/String` | `robot_controller` | desktop client, `mqtt_bridge` |
| `/camera/raw` | `sensor_msgs/CompressedImage` (gray4/gray8) | `webcam_node` | ArUco / subscribers |
| `/camera/color_jpeg` | `sensor_msgs/CompressedImage` (jpeg) | `webcam_node` | `yolo_detection_node`, viewers |
| `/camera/detection` | image (same transport as input) | `yolo_detection_node` | `mqtt_bridge`, desktop |
| `/detections/results` | `std_msgs/String` (JSON) | `yolo_detection_node` | `mqtt_bridge` |
| `camera/annotated` | `sensor_msgs/Image` | `aruco_processor` | viewers |
| `image/coordinates` | `std_msgs/String` (JSON) | `aruco_processor` | desktop client |
| `/coordinates` | `geometry_msgs/PointStamped` | various | desktop client |
| `/mqtt_bridge/status` | `std_msgs/String` | `mqtt_bridge` | monitoring |

> The desktop client subscribes to `camera/raw` as `sensor_msgs/Image`; the production `webcam_node` publishes `CompressedImage`. Use `/camera/color_jpeg` (or a republisher) when feeding the desktop client.

---

## 2. ROS2 services

| Service | Type | Provided by | Effect |
|---------|------|-------------|--------|
| `/laser/set` | `std_srvs/SetBool` | `imu_node` | `data: true` → laser ON, `false` → OFF |

```bash
ros2 service call /laser/set std_srvs/srv/SetBool "{data: true}"
```

---

## 3. MQTT topics (`robot/*`)

| Topic | Source ROS2 topic | Payload |
|-------|-------------------|---------|
| `robot/status` | `/robot_status` | string |
| `robot/odom` | `/odom` | JSON: x, y, theta, vx, heading_deg, timestamp |
| `robot/imu` | `/imu/data` | JSON: heading, roll, pitch, ax/ay/az, gx/gy/gz, timestamp |
| `robot/image` | `/camera/detection` | JSON: base64 image, format, width, height, timestamp |
| `robot/detections` | `/detections/results` | JSON: detections[] + position |

Default broker: `1883`. See the [MQTT & Mobile](#) page for payload examples and rates.

---

## 4. USB serial: Raspberry Pi ↔ Arduino Uno (motor controller)

- **Baud:** 115200, raw mode (`cfmakeraw`), 1 s read timeout.
- **Discovery:** the Pi toggles DTR to reset the Uno, then reads `NODE_ID:motor_controller`.
- **Pi → Uno:** 6-byte binary motor packet `[Dir1, Spd1, Dir2, Spd2, Dir3, Spd3]` (`Dir`: 0=fwd, 1=back, 2=stop; `Spd`: 0–255). Servo angles as text `S1:<deg>\n` / `S2:<deg>\n`.
- **Uno → Pi:** human-readable `DEBUG:` lines (diagnostics only).

---

## 5. USB serial: Raspberry Pi ↔ Arduino Nano (sensor node)

- **Baud:** 115200.
- **Discovery:** the Pi drains 1 s of IMU flood, sends `WHOAMI\n`, expects `NODE_ID:sensor_node`.
- **Nano → Pi:** `A:ax,ay,az G:gx,gy,gz M:mx,my,mz` at ~100 Hz (scaled int16: accel ×1000 g, gyro ×10 dps, mag ×10 µT). `imu_node` parses only `A:` and `G:`.
- **Pi → Nano:** text `LASER_ON\n` / `LASER_OFF\n` / `WHOAMI\n`.

---

## 6. SPI: Raspberry Pi (master) ↔ Arduino Nano (slave)

Alternate transport (not on the production path).

**Master → slave (2 bytes):**

| Byte | Field | Values |
|------|-------|--------|
| 0 | Command | `0x00` off / `0x01` on / `0xFF` NOP |
| 1 | Reserved | ignored |

**Slave → master (12 bytes):** 6× `int16_t` big-endian — `ax, ay, az, gx, gy, gz`.

---

## 7. Unit conventions

| Quantity | On the wire | In ROS2 (`/imu/data`) |
|----------|-------------|------------------------|
| Acceleration | int16, g × 1000 | m/s² (`raw/ACC_SCALE · 9.81`) |
| Angular velocity | int16, dps × 10 | rad/s (`raw/GYR_SCALE · π/180`) |
| Magnetometer | int16, µT × 10 | dropped (no orientation; covariance[0] = −1) |
| Heading (MQTT) | — | degrees 0–360 from quaternion yaw |
