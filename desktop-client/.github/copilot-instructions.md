# Copilot Instructions — Precision Farming Desktop Client

## Big picture (read this first)
- This is a Qt6/C++17 desktop client with optional ROS2 integration; app startup is `src/main.cpp` → `src/core/Application.*`.
- `Application` orchestrates subsystem init in dependency order: `ROS2Interface` → `DigitalTwin` → `MainWindow` + `WidgetManager`.
- UI is modular and dock-based (`QDockWidget`), with widgets created via factory/registry in `src/core/WidgetManager.cpp`.
- Runtime data flow is signal/slot driven: ROS2 callbacks emit Qt signals, widgets/twin consume those signals.

## Core module boundaries
- `src/ros2/ROS2Interface.*`: ROS2 pub/sub bridge + thread boundary. Keep ROS2-specific code behind `#ifdef USE_ROS2` with stub behavior in `#else`.
- `src/twin/DigitalTwin.*`, `TwinState.*`, `TwinSimulator.*`: twin mode/state/simulation; ROS2 data updates `TwinState` when synchronized.
- `src/ui/MainWindow.*`: menus, toolbar, status bar, default dock layout, dependency injection into widgets.
- `src/ui/widgets/*`: feature widgets inheriting `BaseWidget`; each implements `initialize()` and `displayName()`.
- `src/utils/Logger.*`: singleton logger used across all modules (`Logger::instance().info/warning/error`).

## Build and run workflow (preferred)
- Build with scripts at repo root:
  - `./build.sh clean debug`
  - `./build.sh clean release`
- Run with `./run.sh` (auto-builds if executable missing, prints last log lines on exit).
- Manual CMake build is supported but scripts encode project defaults (`-DUSE_ROS2` from `ROS_DISTRO`).
- No test suite is present in this workspace; validate by building + launching app and checking `PrecisionFarmingClient.log`.

## ROS2 and integration facts (code-accurate)
- Published topics: `/cmd_vel`, `/robot_command` (`src/ros2/ROS2Interface.cpp`).
- Subscribed topics: `camera/raw`, `/imu/data`, `/robot_status`, `/coordinates`, `image/coordinates`.
- Camera stream handling converts `bgr8` to RGB before emitting `imageReceived`.
- ROS2 spinning runs on a dedicated `QThread` with a 10 ms `QTimer` (`spin_some` loop).

## Project-specific conventions
- Keep startup resilient when ROS2 is unavailable: app must still initialize in stub mode.
- For new widgets, follow this sequence:
  1. Implement widget in `src/ui/widgets/` deriving `BaseWidget`.
  2. Add enum/factory case + registration in `WidgetManager`.
  3. Add header/source to `CMakeLists.txt`.
  4. Add menu/toolbar hook in `MainWindow` if user-visible.
- `MainWindow::addWidgetToDock()` is where new widgets receive `setROS2Interface()`, `setDigitalTwin()`, then `initialize()`.
- Digital Twin dock creation is intentionally disabled in `MainWindow::onAddTwinVisualization()`; do not re-enable unless requested.

## Editing guardrails for agents
- Prefer minimal, surgical changes; preserve current dock-based architecture unless task explicitly requests UI redesign.
- Treat topic names in code as source of truth over stale docs (some docs still reference `/camera/image_raw`).
- When changing ROS2 callbacks/signals, update all connected widget/twin slots in the same change.
- Keep logs meaningful; this codebase relies on runtime logs for debugging more than automated tests.
