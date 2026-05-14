# `ros2/` — ROS2 Package Documentation

Reference documentation for the ROS2 packages used in the Precision Farming Robot 2.0. The live ROS2 workspaces are in [`raspberry-pi/ros2_robot_ws/`](../raspberry-pi/ros2_robot_ws/) and [`ros2_ws/`](../ros2_ws/).

---

## Package Overview

| Package | Location | Language | Role |
|---------|----------|----------|------|
| `motor_control` | `raspberry-pi/ros2_robot_ws/src/` | C++ | L298N motor driver |
| `imu_sensor` | `raspberry-pi/ros2_robot_ws/src/` | C++ | MPU6050 IMU publisher |
| `encoder_odometry` | `raspberry-pi/ros2_robot_ws/src/` | C++ | Encoder-based odometry |
| `robot_controller` | `raspberry-pi/ros2_robot_ws/src/` | C++ | Control coordinator |
| `camera_sensor` | `raspberry-pi/ros2_robot_ws/src/` | Python | Pi camera / USB webcam |
| `mqtt_bridge` | `raspberry-pi/ros2_robot_ws/src/` | Python | ROS2 ↔ MQTT bridge |
| `yolo_detection` | `raspberry-pi/ros2_robot_ws/src/` | Python | YOLO weed detection |
| `weed_detection` | `ros2_ws/src/` | Python | Weed detection (alt workspace) |

---

## ROS2 Topic Reference

### Core Robot Topics

| Topic | Type | Publisher | Subscriber(s) |
|-------|------|-----------|---------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Desktop client, Nav2 | `motor_control`, `robot_controller` |
| `/cmd_vel_filtered` | `geometry_msgs/Twist` | `robot_controller` | `motor_control` |
| `/imu/data` | `sensor_msgs/Imu` | `imu_sensor` | `robot_controller`, desktop client, `mqtt_bridge` |
| `/odom` | `nav_msgs/Odometry` | `encoder_odometry` | `robot_controller`, `mqtt_bridge` |
| `/robot_status` | `std_msgs/String` | `robot_controller` | Desktop client, `mqtt_bridge` |

### Camera & Vision Topics

| Topic | Type | Publisher | Subscriber(s) |
|-------|------|-----------|---------------|
| `camera/raw` | `sensor_msgs/Image` | `camera_sensor` | `aruco_processor`, `yolo_detection`, desktop client |
| `camera/annotated` | `sensor_msgs/Image` | `aruco_processor` | Desktop client |
| `camera/detection` | `sensor_msgs/Image` | `yolo_detection` | `mqtt_bridge` |
| `image/coordinates` | `std_msgs/String` (JSON) | `aruco_processor` | Desktop client |
| `/detections/results` | `std_msgs/String` (JSON) | `yolo_detection` | `mqtt_bridge` |
| `/coordinates` | `geometry_msgs/PointStamped` | Various | Desktop client |

### MQTT Bridge Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mqtt_bridge/status` | `std_msgs/String` | Bridge connection state (`connected`, `disconnected`, `waiting_for_broker`) |

---

## Node Parameters Reference

### `motor_driver` (motor_control)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheel_base` | `0.2` m | Distance between left and right wheels |
| `wheel_radius` | `0.05` m | Wheel radius |
| `max_speed` | `1.0` m/s | Maximum linear speed |

### `imu_node` (imu_sensor)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_rate` | `50.0` Hz | Sensor publish frequency |
| `i2c_bus` | `1` | I2C bus number |
| `i2c_address` | `0x68` | MPU6050 I2C address |

### `encoder_node` (encoder_odometry)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheel_radius` | `0.05` m | Wheel radius |
| `wheel_base` | `0.2` m | Distance between wheels |
| `counts_per_rev` | `20` | Encoder counts per full revolution |
| `update_rate` | `20.0` Hz | Odometry publish frequency |

### `robot_controller` (robot_controller)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `control_loop_rate` | `20.0` Hz | Main control loop frequency |
| `max_linear_velocity` | `1.0` m/s | Velocity clamp |
| `max_angular_velocity` | `2.0` rad/s | Angular velocity clamp |
| `min_battery_voltage` | `7.0` V | Emergency stop threshold |

### `mqtt_bridge_node` (mqtt_bridge)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mqtt_host` | `localhost` | MQTT broker hostname or IP |
| `mqtt_port` | `1883` | MQTT broker port |
| `mqtt_keepalive` | `60` s | MQTT keepalive interval |
| `odom_rate_hz` | `1.0` | Max odometry publish rate |
| `imu_rate_hz` | `1.0` | Max IMU publish rate |
| `image_rate_hz` | `0.5` | Max camera frame publish rate |
| `detection_rate_hz` | `1.0` | Max detection publish rate |
| `camera_detection_topic` | `/camera/detection` | Source image topic for MQTT |
| `camera_detection_transport` | `auto` | `auto`, `raw`, or `compressed` |
| `image_format` | `png` | Output format: `png` or `jpeg` |
| `image_quality` | `95` | JPEG quality (1–95, ignored for PNG) |

### `webcam_node` (camera_sensor)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device_index` | `0` | V4L2 device index (`/dev/video0` = `0`) |
| `image_width` | `640` | Capture width |
| `image_height` | `480` | Capture height |
| `publish_rate` | `30.0` Hz | Frame publish rate |
| `frame_id` | `camera_link` | TF frame name |
| `camera_topic` | `/camera/raw` | Output topic |

---

## Workspace Setup

### Prerequisites

```bash
# Install ROS2 Jazzy (Ubuntu 24.04)
sudo apt install ros-jazzy-desktop

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Install build tools
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
```

### Build

```bash
cd raspberry-pi/ros2_robot_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build

# Or build specific package
colcon build --packages-select motor_control

# Source the workspace
source install/setup.bash
```

### Launch

```bash
# Launch entire robot stack
ros2 launch robot robot.launch.py

# Or launch individual nodes
ros2 run motor_control motor_driver
ros2 run imu_sensor imu_node
ros2 run encoder_odometry encoder_node
ros2 run robot_controller robot_controller

# Camera
./start_node.sh webcam    # USB webcam
./start_node.sh camera    # Raspberry Pi camera
```

---

## Differential Drive Kinematics

Motor velocities are derived from `/cmd_vel` using:

```
v_left  = v_x − (ω × L / 2)
v_right = v_x + (ω × L / 2)
```

Where `v_x` = linear velocity (m/s), `ω` = angular velocity (rad/s), `L` = wheel base (m).

Wheel angular velocity: `ω_wheel = v / r` where `r` = wheel radius (m).

PWM duty cycle: `pwm = clamp(ω_wheel / ω_max, 0, 1) × 255`

---

## Planned Integrations

- **Nav2** — Autonomous navigation and path planning
- **RViz2** — Real-time 3D visualization
- **TF2** — Coordinate frame management (`base_link`, `odom`, `map`)
- **Kalman filter** — Fused IMU + encoder state estimation
- **Obstacle detection** — Ultrasonic or LiDAR safety layer
