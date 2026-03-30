# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Structure

| Directory | Role |
|-----------|------|
| `desktop-client/` | C++17/Qt6 control & monitoring GUI with optional ROS2 |
| `raspberry-pi/ros2_robot_ws/` | ROS2 Jazzy C++ packages for the physical robot |
| `firmware/` | PlatformIO/Arduino firmware for the ATmega328 (Arduino Nano) |
| `weed-detection-node/` | Python ROS2 node for ArUco marker / weed detection |
| `test_nodes/` | Camera publisher test utility |

## Building Each Component

### Desktop Client (Qt6)

```bash
cd desktop-client
./build.sh clean release   # or: ./build.sh clean debug
./run.sh                   # auto-builds if missing; logs to PrecisionFarmingClient.log
```

Scripts detect `ROS_DISTRO` and set `-DUSE_ROS2` automatically. App runs without ROS2 in stub mode.

### Raspberry Pi (ROS2 Jazzy)

```bash
cd raspberry-pi
./setup.sh --install-deps  # first time: installs gpiozero, pigpio, I2C tools
./setup.sh --build         # colcon build --parallel-workers 1 (memory constrained)
source ros2_robot_ws/install/setup.sh
ros2 launch robot robot.launch.py
```

### Firmware (PlatformIO)

```bash
cd firmware
pio run                     # compile
pio run --target upload     # flash to /dev/ttyUSB0 at 57600 baud
pio device monitor          # serial monitor at 115200
```

## No Automated Tests

Validate desktop client by building, launching, and checking `PrecisionFarmingClient.log`.

Validate robot nodes manually:
```bash
ros2 topic echo /cmd_vel
ros2 topic echo /imu/data
ros2 topic echo /odom
ros2 node list
```

## Overall Architecture

The system is a 4-wheel differential-drive robot. Communication between all subsystems is via ROS2 publish/subscribe.

```
Desktop Client (Qt6 GUI)
│  ROS2Interface (QThread) ──publishes──▶ /cmd_vel, /robot_command
│                           ◀─subscribes─ camera/raw, /imu/data, /robot_status,
│                                          /coordinates, image/coordinates
│  DigitalTwin (Synchronized | Simulated | Offline)
└  WidgetManager (factory for all dock widgets)

Raspberry Pi (ROS2 Jazzy)
├─ robot_controller  — coordination, publishes /robot_status
├─ motor_control     — subscribes /cmd_vel, drives L298N GPIO
├─ imu_sensor        — publishes /imu/data (MPU6050 via I2C)
└─ encoder_odometry  — publishes /odom, /coordinates

Arduino Nano (Firmware)
└─ Low-level PWM/encoder via serial to Raspberry Pi
```

All robot tuning parameters (motor speed, IMU rate, GPIO pin mapping, safety limits) live in:
`raspberry-pi/ros2_robot_ws/config/robot_config.yaml`

## Desktop Client — Key Rules

See `desktop-client/CLAUDE.md` for the full desktop-client guide. Critical rules:

- The app **must** initialize successfully with ROS2 unavailable (stub mode via `#ifdef USE_ROS2`)
- Digital Twin dock is intentionally disabled in `MainWindow::onAddTwinVisualization()`; do not re-enable unless explicitly asked
- ROS2 topic names in code are the source of truth — docs may be stale
- Startup order: `ROS2Interface` → `DigitalTwin` → `MainWindow` + `WidgetManager`
- Adding a new widget: subclass `BaseWidget`, register in `WidgetManager`, add to `CMakeLists.txt`, optionally hook into `MainWindow`
