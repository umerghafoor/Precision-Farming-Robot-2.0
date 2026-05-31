# Desktop Client (`desktop-client/`)

A **Qt6 C++17** desktop application for live robot monitoring, command control, and a 3D **digital twin**. ROS2 integration is optional — without ROS2 sourced it builds and runs in a standalone/simulation mode.

| | |
|--|--|
| Language | C++17 |
| UI toolkit | Qt6 (`Core`, `Gui`, `Widgets`, `Network`; optional `Multimedia`) |
| Build | CMake ≥ 3.16 (`build.sh` / `run.sh` wrappers) |
| Optional | ROS2 (auto-detected via `ROS_DISTRO`) |
| Log file | `PrecisionFarmingClient.log` |

---

## 1. Startup flow

```
src/main.cpp
   └─ Application
        ├─ ROS2Interface     (pub/sub, thread boundary)
        ├─ DigitalTwin       (modes / state / simulation)
        └─ MainWindow + WidgetManager   (menus, docks, widgets)
```

`Application` is the facade that orchestrates lifecycle and dependency injection in this order. `MainWindow::addWidgetToDock()` is where each new widget receives `setROS2Interface()`, `setDigitalTwin()`, then `initialize()`.

---

## 2. Module boundaries (`src/`)

| Path | Responsibility |
|------|----------------|
| `core/Application.*` | Facade; lifecycle + wiring |
| `core/WidgetManager.*` | Factory + registry for all dock widgets |
| `ros2/ROS2Interface.*` | All ROS2 pub/sub; runs on a dedicated `QThread` with a 10 ms `spin_some` timer. ROS2 code is behind `#ifdef USE_ROS2`, with stub behaviour in `#else` |
| `twin/DigitalTwin.*` | Twin orchestration / modes |
| `twin/TwinState.*` | State model |
| `twin/TwinSimulator.*` | Internal physics simulation |
| `ui/MainWindow.*` | Menus, toolbar, default dock layout, ROS2/Twin injection |
| `ui/widgets/*` | Feature widgets; all inherit `BaseWidget` (`initialize()`, `displayName()`) |
| `rendering/*` | OpenGL OBJ model loading/rendering (`OBJLoader`, `OBJMesh`, `ModelGLView`) for the 3D twin |
| `utils/Logger.*` | Singleton logger (`Logger::instance().info/warning/error`) |

---

## 3. Widgets (registered in `WidgetManager`)

Currently registered by default (from `WidgetManager.cpp`):

| Widget | Type key | Description |
|--------|----------|-------------|
| Video Stream | `video` | Live camera feed |
| Controls (Sidebar) | `controls` | Command sidebar |
| Sensor Data | `sensor` | IMU / telemetry readout |
| Current Detection | `current_detection` | Latest detection result |
| Detection Summary | `detection_summary` | Aggregate detection stats |
| Detection Panel | `detection_panel` | Detailed detection list |
| Coordinates | `coordinates` | Live position |
| Digital Twin | `twin` | Twin visualization |
| Robot 3D Model | `model3d` | OpenGL OBJ model viewer |
| Laser Calibration | `laser_cal` | Laser targeting calibration |
| IMU 3D View | `imu3d` | 3D IMU orientation view |

`MotionControl` and `CommandControl` remain in the `WidgetType` enum but their registration is commented out (kept for compatibility / re-enable). Additional widget classes present in `src/ui/widgets/`: `RobotMapView`, `ZoomableImageView`, `StatusBadge` (support/composite widgets).

---

## 4. ROS2 contract (code is source of truth)

### Published
- `/cmd_vel` (`geometry_msgs/Twist`)
- `/robot_command` (`std_msgs/String`)

### Subscribed
- `camera/raw` (`sensor_msgs/Image`)
- `/imu/data` (`sensor_msgs/Imu`)
- `/robot_status` (`std_msgs/String`)
- `/coordinates` (`geometry_msgs/PointStamped`)
- `image/coordinates` (`std_msgs/String`, JSON)

### Notes
- Incoming `bgr8` images are converted to RGB before the `imageReceived` Qt signal is emitted.
- The camera topic can be switched at runtime via `ROS2Interface::switchCameraTopic(...)`.
- ROS2 spinning lives on a dedicated `QThread` (10 ms `QTimer` → `spin_some`).

---

## 5. Digital twin modes

| Mode | Behaviour |
|------|-----------|
| `Synchronized` | mirrors live ROS2 robot state |
| `Simulated` | internal physics simulation (`TwinSimulator`), no robot needed |
| `Offline` | standalone, no data sources |

> `MainWindow::onAddTwinVisualization()` intentionally disables creating the twin dock by default; do not re-enable without an explicit request.

---

## 6. Build & run

```bash
cd desktop-client

# auto-detects ROS2 via ROS_DISTRO, sets -DUSE_ROS2 accordingly
./build.sh clean release      # or: ./build.sh clean debug
./run.sh                       # auto-builds if missing; prints last log lines on exit
```

Manual CMake:

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_ROS2=ON
make -j"$(nproc)"
```

To build **without** ROS2, leave `ROS_DISTRO` unset before `./build.sh` (it sets `-DUSE_ROS2=OFF`).

There is no automated test suite — validate by building, launching, and inspecting `PrecisionFarmingClient.log`. A `Dockerfile` + `build_docker.sh` / `start_docker.sh` are provided for a containerised build, and `setup_ros2_pi.sh` / `install_dependency.sh` help provision dependencies.

---

## 7. Extending

**Add a widget:**
1. Create a class deriving `BaseWidget` in `src/ui/widgets/`.
2. Add an enum case + `registerWidget(...)` + factory case in `WidgetManager`.
3. Add the source/header to `CMakeLists.txt`.
4. Add a menu/toolbar action in `MainWindow` if user-visible.

**Add a ROS2 topic:**
1. Add the publisher/subscriber in `ROS2Interface` (inside `#ifdef USE_ROS2`).
2. Bridge data through Qt signals.
3. Consume it in a widget and/or `DigitalTwin`. Update all connected slots in the same change.

**House rules:** the app must initialise successfully with ROS2 unavailable; topic names in code win over docs; prefer surgical changes that preserve the dock-based architecture.

> The desktop client ships its own deeper design docs under `desktop-client/docs/` (`ARCHITECTURE.md`, `DIAGRAMS.md`, `FILE_STRUCTURE.md`, `ROS2-Communication.md`, `ROS2_NODES.md`, `PROJECT_SUMMARY.md`, `QUICKSTART.md`).
