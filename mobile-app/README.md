# `mobile-app/` — Mobile Control & Monitoring App

Mobile application for remote control and real-time monitoring of the Precision Farming Robot 2.0. Connects to the robot via MQTT over Wi-Fi.

---

## Overview

The mobile app is the field-operator interface. It receives live telemetry from the robot via the `mqtt_bridge` ROS2 node and allows sending motion commands without requiring a laptop or ROS2 installation on the operator's device.

---

## MQTT Topics

The app communicates with the robot through an MQTT broker (default: port 1883). All data is published by the `mqtt_bridge` ROS2 node running on the Raspberry Pi.

### Subscribed (robot → app)

| Topic | Payload | Description |
|-------|---------|-------------|
| `robot/status` | String | Robot operational status |
| `robot/odom` | JSON | Position `x`, `y`, heading `theta` (rad), `heading_deg` (0–360°), velocity `vx`, `timestamp` |
| `robot/imu` | JSON | `heading`, `roll`, `pitch` (degrees), `ax`, `ay`, `az` (m/s²), `gx`, `gy`, `gz` (rad/s), `timestamp` |
| `robot/image` | JSON | Base64-encoded camera frame (`image_b64`), `width`, `height`, `image_format` (png/jpeg), `timestamp` |
| `robot/detections` | JSON | Weed/marker detection results with robot position annotation |

### Published (app → robot)

Motion commands should be sent to the ROS2 stack. This can be done by:
- Publishing directly to `/cmd_vel` if the app has a ROS2 bridge
- Sending commands via a custom MQTT → ROS2 bridge topic (extend `mqtt_bridge` as needed)

---

## Robot Odometry Payload Example

```json
{
  "x": 1.2345,
  "y": 0.8765,
  "theta": 1.5708,
  "vx": 0.25,
  "heading_deg": 90.0,
  "timestamp": 1715000000
}
```

## IMU Payload Example

```json
{
  "heading": 90.0,
  "roll": 1.2,
  "pitch": -0.5,
  "ax": 0.01,
  "ay": -0.02,
  "az": 9.81,
  "gx": 0.001,
  "gy": 0.002,
  "gz": -0.001,
  "timestamp": 1715000000
}
```

## Camera Image Payload Example

```json
{
  "image_b64": "<base64-encoded PNG or JPEG>",
  "image_png_b64": "<same as image_b64 for PNG>",
  "image_encoding": "base64",
  "image_format": "png",
  "mime_type": "image/png",
  "width": 640,
  "height": 480,
  "timestamp": 1715000000
}
```

---

## MQTT Broker Setup

The MQTT broker must be reachable from both the Raspberry Pi and the mobile device on the same network.

**Run a local broker on the Raspberry Pi:**

```bash
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

**Configure the bridge node to connect:**

```bash
ros2 run mqtt_bridge mqtt_bridge_node \
  --ros-args \
  -p mqtt_host:=<raspberry_pi_ip> \
  -p mqtt_port:=1883 \
  -p image_rate_hz:=0.5 \
  -p odom_rate_hz:=1.0 \
  -p imu_rate_hz:=1.0 \
  -p image_format:=jpeg \
  -p image_quality:=80
```

**Test MQTT from a terminal:**

```bash
# Subscribe to all robot topics
mosquitto_sub -h <broker_ip> -t "robot/#" -v

# Check bridge status
mosquitto_sub -h <broker_ip> -t "robot/status"
```

---

## Planned Features

- Live video stream from `robot/image`
- Joystick / D-pad motion control
- Real-time IMU orientation display (pitch, roll, heading)
- Field map with robot position from `robot/odom`
- Detection results overlay from `robot/detections`
- Connection status indicator (MQTT bridge health)
- Configuration: MQTT host/port, image quality, update rates

---

## Development Setup

*Platform and framework to be confirmed. Setup and build instructions will be added here.*

### Prerequisites (planned)

- Flutter / React Native / native Android or iOS SDK
- MQTT client library (`paho-mqtt`, `mqtt.js`, or platform equivalent)
- Network access to the MQTT broker

---

## Troubleshooting

| Problem | Check |
|---------|-------|
| No data received | Verify `mosquitto_sub -h <ip> -t "robot/#"` receives messages |
| Bridge not connecting | `ros2 topic echo /mqtt_bridge/status` — should show `connected` |
| Images not arriving | `image_rate_hz` defaults to 0.5 Hz (one frame every 2 s); increase if needed |
| High latency | Reduce `image_quality` or switch to `image_format:=jpeg` |
