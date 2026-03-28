# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
# Clean release build
./build.sh clean release

# Clean debug build
./build.sh clean debug

# Run (auto-builds if executable missing; prints last log lines on exit)
./run.sh
```

The scripts detect `ROS_DISTRO` and set `-DUSE_ROS2` automatically. Manual CMake is supported but the scripts encode project defaults.

There is no test suite. Validate by building, launching the app, and checking `PrecisionFarmingClient.log`.

## Architecture

Startup flow: `src/main.cpp` → `Application` → subsystems initialized in dependency order: `ROS2Interface` → `DigitalTwin` → `MainWindow` + `WidgetManager`.

**Module boundaries:**
- `src/core/Application.*` — Facade; orchestrates lifecycle and dependency injection
- `src/core/WidgetManager.*` — Factory + registry for all dock widgets
- `src/ros2/ROS2Interface.*` — All ROS2 pub/sub logic; thread boundary. ROS2-specific code lives behind `#ifdef USE_ROS2` with stub behavior in `#else`
- `src/twin/` — Digital Twin: `DigitalTwin` (modes/orchestration), `TwinState` (state model), `TwinSimulator` (physics sim)
- `src/ui/MainWindow.*` — Menus, toolbar, default dock layout, injects ROS2/Twin into widgets
- `src/ui/widgets/` — Feature widgets; all inherit `BaseWidget`, implement `initialize()` and `displayName()`
- `src/utils/Logger.*` — Singleton logger (`Logger::instance().info/warning/error(...)`)

**ROS2 topics (code is source of truth; some docs are stale):**
- Published: `/cmd_vel` (Twist), `/robot_command` (String)
- Subscribed: `camera/raw`, `/imu/data`, `/robot_status`, `/coordinates`, `image/coordinates`
- Camera: `bgr8` → RGB8 conversion before emitting `imageReceived`
- ROS2 spins on a dedicated `QThread` with a 10 ms `QTimer` (`spin_some` loop)

**Digital Twin modes:** `Synchronized` (mirrors ROS2), `Simulated` (internal physics), `Offline` (standalone)

## Conventions

**Adding a new widget:**
1. Implement in `src/ui/widgets/` deriving `BaseWidget`
2. Add enum case + factory/registration in `WidgetManager`
3. Add header/source to `CMakeLists.txt`
4. Add menu/toolbar hook in `MainWindow` if user-visible

`MainWindow::addWidgetToDock()` is where new widgets receive `setROS2Interface()`, `setDigitalTwin()`, then `initialize()`.

**Other rules:**
- App must initialize successfully with ROS2 unavailable (stub mode)
- Digital Twin dock creation is intentionally disabled in `MainWindow::onAddTwinVisualization()`; do not re-enable unless explicitly requested
- When changing ROS2 callbacks/signals, update all connected widget/twin slots in the same change
- Prefer surgical changes; preserve dock-based architecture unless a UI redesign is explicitly requested
- Treat topic names in code as source of truth over docs
