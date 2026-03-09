# Precision Farming Robot — Desktop Client

Qt6 desktop app for robot control, telemetry, and simulation, with optional ROS2 integration.

## TL;DR

- Stack: C++17 + Qt6 (`Core`, `Gui`, `Widgets`, `Network`), optional Qt Multimedia, optional ROS2.
- Entry flow: `main.cpp` → `Application` → `ROS2Interface` → `DigitalTwin` → `MainWindow`.
- UI model: dockable widget workspace managed by `WidgetManager`.
- ROS2 mode is optional: if ROS2 is not sourced/found, app builds and runs in stub/standalone mode.

## Core Capabilities

- Modular dockable widgets (add/remove/rearrange at runtime).
- Robot command publishing (`/cmd_vel`, `/robot_command`).
- Real-time telemetry subscriptions (camera, IMU, status, coordinates).
- Digital Twin modes:
  - `Synchronized` (mirrors ROS2 updates)
  - `Simulated` (internal simulation)
  - `Offline`
- Structured logging to `PrecisionFarmingClient.log`.

## Widget Set (Current)

Registered by default in `WidgetManager`:

- `Video Stream`
- `Motion Control`
- `Command & Control`
- `Sensor Data`
- `Coordinates`
- `Digital Twin` (type exists, but UI creation is currently disabled in `MainWindow`)

## ROS2 Contract (Code-Accurate)

### Published

- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/robot_command` (`std_msgs/msg/String`)

### Subscribed

- `camera/raw` (`sensor_msgs/msg/Image`)
- `/imu/data` (`sensor_msgs/msg/Imu`)
- `/robot_status` (`std_msgs/msg/String`)
- `/coordinates` (`geometry_msgs/msg/PointStamped`)
- `image/coordinates` (`std_msgs/msg/String`, JSON payload)

### Notes

- Image encoding handling: if incoming encoding is `bgr8`, it is converted to RGB before Qt signal emission.
- ROS2 spinning runs on a dedicated `QThread` with a `QTimer` at 10 ms (`spin_some` loop).
- Camera topic can be switched dynamically via `ROS2Interface::switchCameraTopic(...)`.

## Build

From `desktop-client/`:

```bash
./build.sh clean debug
# or
./build.sh clean release
```

What `build.sh` does:

- Detects ROS2 using `ROS_DISTRO`.
- Sets `-DUSE_ROS2=OFF` automatically when ROS2 is not sourced.
- Configures CMake and builds `build/PrecisionFarmingDesktopClient`.

### Manual CMake (optional)

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_ROS2=ON
make -j"$(nproc)"
```

## Run

```bash
./run.sh
```

- Auto-builds if executable is missing.
- Runs in standalone mode when ROS2 is unavailable.
- Prints last log entries on exit.

## Runtime UX Summary

- Main window starts with a default dock layout (video/coordinates left, controls right, sensor bottom).
- Menu groups: `File`, `Widgets`, `ROS2`, `Simulation`, `Help`.
- ROS2 connection is toggled from the UI (`Connect` action).
- Simulation is toggled from the UI (`Start Simulation` action).

## Project Map

- `src/main.cpp` — Qt app bootstrap + logger init.
- `src/core/Application.*` — lifecycle orchestration and dependency wiring.
- `src/core/WidgetManager.*` — widget registration/factory/active instances.
- `src/ros2/ROS2Interface.*` — ROS2 pub/sub + Qt bridge.
- `src/twin/DigitalTwin.*` — twin mode/state and simulation control.
- `src/ui/MainWindow.*` — menus, docking, default layout.
- `src/ui/widgets/*` — per-widget UI and behavior.
- `src/utils/Logger.*` — centralized logging.

## Extension Checklist

### Add a new widget

1. Create widget class deriving from `BaseWidget`.
2. Register enum/name and factory case in `WidgetManager`.
3. Add source/header to `CMakeLists.txt`.
4. Expose menu/toolbar action in `MainWindow` if needed.

### Add a ROS2 topic

1. Add publisher/subscriber in `ROS2Interface`.
2. Bridge data through Qt signals.
3. Consume in widgets and/or `DigitalTwin`.

## Requirements

- Linux (Ubuntu recommended)
- CMake >= 3.16
- C++17 toolchain
- Qt6 base development packages
- Optional: ROS2 (for live robot integration)

## Troubleshooting

- Build without ROS2 intentionally: leave ROS2 unsourced, then run `./build.sh`.
- If camera appears wrong-color, verify source encoding (`bgr8` vs `rgb8`).
- Check logs in `PrecisionFarmingClient.log` for startup/init/connectivity issues.
