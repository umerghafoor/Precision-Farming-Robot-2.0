# Copilot Instructions for Precision-Farming-Robot-2.0

## Scope and source of truth
- This repo is multi-component; most active implementation is in `desktop-client/` (Qt6 + optional ROS2).
- Prefer code over stale docs when they disagree. Example: camera topic in code is `camera/raw` (`desktop-client/src/ros2/ROS2Interface.cpp`), while some docs still mention `/camera/image_raw`.

## Big-picture architecture (desktop client)
- Entry point: `desktop-client/src/main.cpp` sets Qt Fusion style, initializes `Logger`, then creates `Application` facade.
- Orchestration: `desktop-client/src/core/Application.cpp` initializes in strict order: `ROS2Interface` → `DigitalTwin` → `MainWindow`/`WidgetManager`.
- Service boundaries:
  - `src/ros2/ROS2Interface.*`: ROS2 node lifecycle + pub/sub bridge to Qt signals.
  - `src/twin/DigitalTwin.*`: twin state/simulation modes (`Synchronized`, `Simulated`, `Offline`).
  - `src/ui/` + `src/ui/widgets/`: dockable widget UI, wired through `BaseWidget`.
- Widget factory pattern: add/instantiate widgets via `WidgetManager::WidgetType` and `createWidget()` in `src/core/WidgetManager.cpp`.

## ROS2 data flow and integration contracts
- UI-to-robot commands:
  - Publishes `/cmd_vel` (`geometry_msgs/msg/Twist`) and `/robot_command` (`std_msgs/msg/String`).
- Robot-to-UI telemetry:
  - Subscribes `camera/raw`, `/imu/data`, `/robot_status`, `/coordinates`, `image/coordinates`.
- Threading model: ROS2 spins via `QThread` + `QTimer` (10 ms) calling `rclcpp::spin_some`; communication to UI is via Qt signals.
- Image handling contract: if incoming encoding is `bgr8`, conversion to RGB is performed before `imageReceived` signal emission.

## Build and run workflows developers actually use
- Desktop client (without Docker):
  - `cd desktop-client && ./build.sh clean debug` or `./build.sh clean release`
  - `./run.sh` (auto-builds if missing executable).
- ROS2 optionality is intentional:
  - `build.sh` sets `USE_ROS2=OFF` when `ROS_DISTRO` is not sourced; code must keep stub mode behavior working.
- CMake details to preserve:
  - C++17, Qt6 Core/Gui/Widgets/Network required; Qt Multimedia optional (`HAVE_QT_MULTIMEDIA`).

## Cross-component notes
- Test camera publisher: `test_nodes/camera_pub_raw/main.py` publishes to `camera/raw` at configurable rate.
- Raspberry Pi ROS2 workspace docs live in `raspberry-pi/ros2_robot_ws/README.md` and define expected robot topics (`/cmd_vel`, `/imu/data`, `/robot_status`, etc.).
- Firmware is separate from ROS2 desktop path:
  - `firmware/src/main.cpp` reads serial commands; `firmware/platformio.ini` targets Arduino Nano (`nanoatmega328`).

## Project-specific coding conventions
- Keep module separation intact (`core`, `ros2`, `twin`, `ui`, `utils`); avoid putting ROS2 logic directly in widgets.
- Use Qt signal/slot bridging for cross-thread communication; do not update Qt widgets from ROS2 callbacks directly.
- New widgets should:
  - inherit `BaseWidget`,
  - be registered in `WidgetManager`,
  - be added to `desktop-client/CMakeLists.txt` sources/headers.
- Logging convention: use `Logger::instance()` consistently for lifecycle/events/errors.
