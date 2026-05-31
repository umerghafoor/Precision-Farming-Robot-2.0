# MQTT Bridge & Mobile App

The robot talks to the **mobile app** through MQTT. The `mqtt_bridge_node` (ROS2, Python) re-publishes selected ROS2 topics to `robot/*` MQTT topics, throttling high-rate data so a phone (and a downstream database) can keep up.

---

## 1. `mqtt_bridge_node` (`raspberry-pi/.../mqtt_bridge/mqtt_bridge_node.py`)

### ROS2 → MQTT mapping

| ROS2 topic | Type | MQTT topic | Notes |
|------------|------|-----------|-------|
| `/robot_status` | String | `robot/status` | passthrough |
| `/detections/results` | String (JSON) | `robot/detections` | JSON + cached odom position, throttled |
| `/odom` | Odometry | `robot/odom` | x, y, theta, vx, heading_deg, timestamp |
| `/camera/detection` | Image / CompressedImage | `robot/image` | base64 image, throttled |
| `/imu/data` | Imu | `robot/imu` | heading, roll, pitch, accel, gyro, timestamp |

It also publishes its own health on `/mqtt_bridge/status` (`connected`, `waiting_for_broker`, `paho_missing`, `disconnected:<rc>`, …) every second.

### Throttling & rates

Each stream is rate-limited to its own minimum interval (`1/rate_hz`). Detections are annotated with the **last known odometry position** so the backend can geo-tag them.

| Parameter | Default |
|-----------|---------|
| `mqtt_host` | `localhost` (launch sets a Tailscale host) |
| `mqtt_port` | 1883 |
| `mqtt_keepalive` | 60 |
| `camera_detection_topic` | `/camera/detection` |
| `camera_detection_transport` | `auto` (`raw` / `compressed`) |
| `odom_rate_hz` | 1.0 |
| `image_rate_hz` | 0.5 (one frame every 2 s) |
| `imu_rate_hz` | 1.0 |
| `detection_rate_hz` | 1.0 |
| `image_format` | `png` (or `jpeg`) |
| `image_quality` | 95 (JPEG only) |

### Robustness

- Uses `paho-mqtt` (`connect_async` + `loop_start`, supports both v1 and v2 callback APIs). If `paho-mqtt` is missing, the node still spins but does not publish.
- Image bridging needs `numpy` + `Pillow`; raw `Image` frames are converted to RGB and re-encoded, compressed JPEG/PNG frames are passed through (or re-encoded if an unknown format).
- Camera transport is auto-detected from the live topic types unless forced.

---

## 2. MQTT payloads (robot → app)

**`robot/odom`**
```json
{ "x": 1.2345, "y": 0.8765, "theta": 1.5708, "vx": 0.25, "heading_deg": 90.0, "timestamp": 1715000000 }
```

**`robot/imu`**
```json
{ "heading": 90.0, "roll": 1.2, "pitch": -0.5,
  "ax": 0.01, "ay": -0.02, "az": 9.81,
  "gx": 0.001, "gy": 0.002, "gz": -0.001, "timestamp": 1715000000 }
```

**`robot/image`**
```json
{ "image_b64": "<base64 PNG/JPEG>", "image_png_b64": "<same for png>",
  "image_encoding": "base64", "image_format": "png", "mime_type": "image/png",
  "width": 640, "height": 480, "timestamp": 1715000000 }
```

**`robot/detections`** — the YOLO results JSON plus a `position` object copied from the latest odom.

Roll/pitch/heading are derived from the IMU quaternion (heading normalised to 0–360°).

---

## 3. Broker setup

```bash
# on the Raspberry Pi (or any reachable host)
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto

# point the bridge at the broker
ros2 run mqtt_bridge mqtt_bridge_node --ros-args \
  -p mqtt_host:=<broker_ip> -p mqtt_port:=1883 \
  -p image_rate_hz:=0.5 -p odom_rate_hz:=1.0 -p imu_rate_hz:=1.0 \
  -p image_format:=jpeg -p image_quality:=80

# verify from any machine on the network
mosquitto_sub -h <broker_ip> -t "robot/#" -v
ros2 topic echo /mqtt_bridge/status     # should read "connected"
```

The default launch host is a **Tailscale** hostname (`sanilinux.mullet-bull.ts.net`), so the broker is reachable across networks via the tailnet.

---

## 4. Mobile app (`mobile-app/`)

The mobile app is the field-operator interface — it consumes the `robot/*` MQTT topics so an operator needs neither a laptop nor a ROS2 install.

### Subscribed (robot → app)

| Topic | Payload |
|-------|---------|
| `robot/status` | operational status string |
| `robot/odom` | position + heading JSON |
| `robot/imu` | orientation + accel/gyro JSON |
| `robot/image` | base64 camera frame JSON |
| `robot/detections` | detection results + position |

### Published (app → robot)

Motion commands go back to the ROS2 stack — either by publishing `/cmd_vel` directly (if the app has a ROS2 bridge) or via a custom MQTT→ROS2 topic added to `mqtt_bridge` (not yet implemented).

### Status

The app is currently a **specification** (no platform/framework committed yet — Flutter / React Native / native are all candidates). Planned features: live video, joystick/D-pad control, real-time IMU orientation, field map from odom, detection overlay, connection-health indicator, and configurable host/quality/rates.

| Problem | Check |
|---------|-------|
| No data received | `mosquitto_sub -h <ip> -t "robot/#"` receives messages? |
| Bridge not connecting | `/mqtt_bridge/status` should show `connected` |
| Images not arriving | `image_rate_hz` defaults to 0.5 Hz; raise it |
| High latency | lower `image_quality` or use `image_format:=jpeg` |
