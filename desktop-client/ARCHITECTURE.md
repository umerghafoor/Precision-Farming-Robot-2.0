# Architecture Documentation

## System Architecture

### High-Level Overview

```txt
┌───────────────────────────────────────────────────────────────┐
│                    Qt Application Layer                       │
│                                                               │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │              Application (Facade)                       │  │
│  │  - Initialization & Orchestration                       │  │
│  │  - Component Lifecycle Management                       │  │
│  └──────────┬──────────────┬──────────────┬────────────────┘  │
│             │              │              │                   │
│    ┌────────▼────────┐ ┌──▼──────────┐ ┌─▼────────────────┐   │
│    │  ROS2Interface  │ │DigitalTwin  │ │   MainWindow     │   │
│    │   (Thread)      │ │   Module    │ │  WidgetManager   │   │
│    └─────────────────┘ └─────────────┘ └──────────────────┘   │
│                                                               │
└───────────────────────────────────────────────────────────────┘
         │                      │                    │
         │                      │                    │
    ┌────▼─────┐          ┌────▼─────┐         ┌───▼─────┐
    │   ROS2   │          │ Physics  │         │   Qt    │
    │  Runtime │          │Simulator │         │  Widgets│
    └──────────┘          └──────────┘         └─────────┘
```

## Module Descriptions

### 1. Core Module (`src/core/`)

#### Application Class

- **Role:** Facade and main orchestrator
- **Responsibilities:**
  - Initialize all subsystems
  - Manage component lifecycle
  - Coordinate inter-module communication
- **Design Pattern:** Facade Pattern
- **Key Methods:**
  - `initialize()` - Bootstrap all components
  - `show()` - Display main window
  - Signal handlers for system events

#### WidgetManager Class

- **Role:** Widget factory and registry
- **Responsibilities:**
  - Create widget instances
  - Track active widgets
  - Manage widget lifecycle
- **Design Patterns:** Factory + Registry
- **Key Methods:**
  - `createWidget(type)` - Factory method
  - `registerWidget()` - Register new types
  - `getWidget(id)` - Retrieve instances

### 2. ROS2 Module (`src/ros2/`)

#### ROS2Interface Class

- **Role:** ROS2 communication layer
- **Responsibilities:**
  - Manage ROS2 node lifecycle
  - Publish/subscribe to topics
  - Thread-safe communication with Qt
- **Threading:** Runs in separate QThread
- **Design Pattern:** Bridge Pattern (Qt ↔ ROS2)

**Thread Safety:**

```cpp
// ROS2 spins in separate thread
void ROS2Interface::spinROS2() {
    rclcpp::spin_some(m_node);  // Non-blocking
}

// Qt signals cross thread boundary safely
emit imageReceived(data, w, h);  // Thread-safe
```

**Topic Architecture:**

```txt
Publishers:
  /cmd_vel          → Velocity commands
  /robot_command    → Custom commands

Subscribers:
  /camera/image_raw ← Video feed
  /imu/data         ← Inertial data
  /robot_status     ← Status updates
```

### 3. Digital Twin Module (`src/twin/`)

#### DigitalTwin Class

- **Role:** Main twin controller
- **Responsibilities:**
  - Manage twin operating modes
  - Synchronize with ROS2
  - Control simulation
- **Design Pattern:** Strategy Pattern (modes)

**Operating Modes:**

```cpp
enum class Mode {
    Synchronized,  // Mirror real robot via ROS2
    Simulated,     // Run physics simulation
    Offline        // Disconnected state
};
```

#### TwinState Class

- **Role:** State container
- **Responsibilities:**
  - Store robot state (pose, velocity, sensors)
  - Emit change notifications
  - Serialize/deserialize state
- **Design Pattern:** Observer Pattern

**State Structure:**

```cpp
struct {
    Pose {
        QVector3D position;
        QQuaternion orientation;
    }
    Velocity {
        QVector3D linear;
        QVector3D angular;
    }
    SensorData {
        QVariantMap imu;
        QVariantMap gps;
        QVariantMap camera;
    }
    double batteryLevel;
    QString robotStatus;
}
```

#### TwinSimulator Class

- **Role:** Physics simulation engine
- **Responsibilities:**
  - Update physics state
  - Simulate sensors
  - Run at configurable rate
- **Design Pattern:** Command Pattern

**Simulation Loop:**

```txt
┌─────────────────────┐
│ Timer (50Hz default)│
└──────────┬──────────┘
           │
    ┌──────▼──────┐
    │   Update    │
    │ Physics     │
    │ (Δt based)  │
    └──────┬──────┘
           │
    ┌──────▼──────┐
    │  Simulate   │
    │  Sensors    │
    └──────┬──────┘
           │
    ┌──────▼──────┐
    │  Emit State │
    │   Changed   │
    └─────────────┘
```

### 4. UI Module (`src/ui/`)

#### MainWindow Class

- **Role:** Main application window
- **Responsibilities:**
  - Manage window layout
  - Coordinate widgets
  - Handle user actions
- **Design Pattern:** Mediator Pattern

**Layout System:**

```txt
┌─────────────────────────────────────────┐
│           Menu Bar                      │
├─────────────────────────────────────────┤
│           Tool Bar                      │
├─────────────────────────────────────────┤
│                                         │
│  ┌──────────┐  ┌──────────┐             │
│  │ Dock     │  │ Dock     │  Central    │
│  │ Widget 1 │  │ Widget 2 │  Widget     │
│  └──────────┘  └──────────┘             │
│                                         │
├─────────────────────────────────────────┤
│           Status Bar                    │
└─────────────────────────────────────────┘
```

#### BaseWidget Class (Abstract)

- **Role:** Widget interface
- **Responsibilities:**
  - Define widget contract
  - Manage connections to data sources
  - Handle lifecycle events
- **Design Pattern:** Template Method Pattern

**Widget Lifecycle:**

```txt
Create → setROS2Interface() → setDigitalTwin() 
       → initialize() → [Active] → closeEvent() → Destroy
```

#### Concrete Widgets

1. **VideoStreamWidget**
   - Display camera feeds
   - Record video (planned)
   - Switch between cameras

2. **CommandControlWidget**
   - Velocity control (sliders)
   - Emergency stop
   - Custom commands

3. **SensorDataWidget**
   - Tabular sensor display
   - Real-time updates (10Hz)
   - Multiple sensor types

4. **TwinVisualizationWidget**
   - Display twin state
   - Mode switching
   - State inspection

### 5. Utils Module (`src/utils/`)

#### Logger Class

- **Role:** Centralized logging
- **Responsibilities:**
  - File and console logging
  - Thread-safe operation
  - Log level filtering
- **Design Pattern:** Singleton Pattern

**Log Levels:**

```txt
Debug → Info → Warning → Error → Critical
```

## Data Flow

### ROS2 → UI Flow

```txt
ROS2 Topic → ROS2Interface::callback()
           → emit signal (crosses thread)
           → Widget::slot()
           → Update UI (Qt thread)
```

### UI → ROS2 Flow

```txt
User Action → Widget::slot()
            → ROS2Interface::publish()
            → ROS2 Publisher (separate thread)
            → ROS2 Topic
```

### Digital Twin Sync Flow

```txt
ROS2 Data → ROS2Interface → emit signal
          → DigitalTwin::onROS2Data()
          → TwinState::update()
          → emit stateChanged
          → TwinVisualizationWidget::update()
```

## Threading Model

```txt
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│  Main/Qt     │   │  ROS2 Spin   │   │  Simulation  │
│  Thread      │   │  Thread      │   │  (Qt Thread) │
└──────┬───────┘   └──────┬───────┘   └──────┬───────┘
       │                  │                   │
       │  Signal/Slot     │                   │
       │◄─────────────────┤                   │
       │                  │                   │
       │  Signal/Slot     │                   │
       │◄─────────────────┼───────────────────┤
       │                  │                   │
```

**Thread Safety Measures:**

- Qt signals/slots (queued connections)
- QMutex in Logger
- ROS2 runs in isolated thread
- No direct cross-thread calls

## Extensibility Points

### Adding New Widgets

1. Inherit from `BaseWidget`
2. Implement `initialize()` and `displayName()`
3. Register in `WidgetManager`
4. Add to factory method

### Adding ROS2 Topics

1. Add publisher/subscriber in `ROS2Interface`
2. Create callback method
3. Emit Qt signal with data
4. Connect in interested widgets

### Adding Twin Features

1. Extend `TwinState` with new data
2. Update `TwinSimulator` if needed
3. Modify `DigitalTwin` mode logic
4. Update visualization widgets

## Performance Considerations

1. **ROS2 Thread:** Non-blocking spin (spin_some)
2. **Video Streaming:** Image conversion in callback
3. **Simulation:** Configurable update rate
4. **UI Updates:** Throttled (10-50 Hz typical)
5. **Logging:** Buffered writes

## Memory Management

- Smart pointers (`std::unique_ptr`) for ownership
- Qt parent-child for widgets
- RAII for resources
- No manual delete needed

## Error Handling

```cpp
try {
    // Operation
} catch (const std::exception& e) {
    Logger::instance().error(message);
    emit errorOccurred(message);
}
```

## Future Architecture Improvements

1. **Plugin System:** Dynamic widget loading
2. **State Machine:** Formal state management
3. **Data Recording:** Time-series database
4. **Distributed System:** Multi-robot support
5. **3D Rendering:** OpenGL/Qt3D integration

---

This architecture provides:

- ✅ Modularity
- ✅ Scalability
- ✅ Testability
- ✅ Maintainability
- ✅ Extensibility
