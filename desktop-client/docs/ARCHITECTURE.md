# Architecture Documentation

## System Architecture

### Overall System Overview

The Precision Farming Robot 2.0 is a distributed robotic system with the following architecture:

```txt
┌─────────────────────────────────────────────────────────────────┐
│                      User Interfaces                            │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐  │
│  │ Desktop Client   │  │   Mobile App     │  │   Web UI     │  │
│  │   (Qt6/C++)      │  │   (Planned)      │  │  (Planned)   │  │
│  └────────┬─────────┘  └────────┬─────────┘  └──────┬───────┘  │
└───────────┼──────────────────────┼────────────────────┼─────────┘
            │                      │                    │
            └──────────────────────┼────────────────────┘
                                   │
                          ┌────────▼────────┐
                          │   ROS2 Network  │
                          │   (DDS Layer)   │
                          └────────┬────────┘
                                   │
         ┌─────────────────────────┼─────────────────────────┐
         │                         │                         │
    ┌────▼──────┐         ┌───────▼────────┐       ┌───────▼──────┐
    │ Digital   │         │  Raspberry Pi  │       │   External   │
    │  Twin     │         │  Robot Brain   │       │   Services   │
    │Simulation │         │   (ROS2 Jazzy) │       │   (Cloud)    │
    └───────────┘         └───────┬────────┘       └──────────────┘
                                  │
                    ┌─────────────┼─────────────┐
                    │             │             │
              ┌─────▼──────┐ ┌───▼────┐ ┌─────▼──────┐
              │   Motors   │ │Sensors │ │  Encoders  │
              │ (L298N x4) │ │(MPU6050)│ │  (x4)     │
              └────────────┘ └────────┘ └────────────┘
```

### Desktop Client High-Level Architecture

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

### Cross-Component Integration

```txt
┌──────────────────────────────────────────────────────────────┐
│                        Desktop Client                        │
│                      (Local or Remote)                       │
└─────────────────┬────────────────────────────────────────────┘
                  │ ROS2 Topics
                  │ (TCP/UDP via DDS)
                  │
┌─────────────────▼────────────────────────────────────────────┐
│                    Raspberry Pi Robot                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │    Motor     │  │     IMU      │  │   Encoder    │       │
│  │   Control    │◄─┤    Sensor    │  │   Odometry   │       │
│  │              │  │              │  │              │       │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘       │
│         │                 │                 │               │
│         └─────────────────┼─────────────────┘               │
│                           │                                 │
│                  ┌────────▼────────┐                        │
│                  │     Robot       │                        │
│                  │   Controller    │                        │
│                  │  (Coordinator)  │                        │
│                  └─────────────────┘                        │
│                           │                                 │
│                  ┌────────▼────────┐                        │
│                  │   GPIO/I2C      │                        │
│                  │   Hardware      │                        │
│                  └─────────────────┘                        │
└──────────────────────────────────────────────────────────────┘
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

## Complete System Integration

### Multi-Component Communication

The complete Precision Farming Robot 2.0 system integrates multiple software components:

#### 1. Desktop Client → Robot Communication

```txt
Desktop Client                 ROS2 DDS                Raspberry Pi
┌─────────────┐               ┌─────────┐             ┌──────────────┐
│ROS2Interface│──publish───►  │/cmd_vel │  ──subscribe─►│motor_driver │
│             │               └─────────┘             │             │
│             │               ┌─────────┐             │             │
│             │◄─subscribe──  │/imu/data│  ◄──publish──│imu_node     │
│             │               └─────────┘             │             │
│             │               ┌─────────┐             │             │
│             │◄─subscribe──  │/odom    │  ◄──publish──│encoder_node │
└─────────────┘               └─────────┘             └──────────────┘
```

#### 2. Data Flow Patterns

**Control Flow** (User → Robot):
```
User Input → Widget → ROS2Interface → /cmd_vel → robot_controller 
  → motor_driver → GPIO → Motors
```

**Sensor Flow** (Robot → User):
```
Sensors → GPIO/I2C → imu_node/encoder_node → ROS2 Topics 
  → ROS2Interface → Widgets → UI Display
```

**Digital Twin Sync**:
```
Robot State → ROS2 Topics → ROS2Interface → DigitalTwin 
  → TwinState → TwinVisualizationWidget
```

#### 3. Network Topologies

**Local Development**:
```
Desktop (localhost) ←→ ROS2 Loopback ←→ Digital Twin Simulator
```

**Production Deployment**:
```
Desktop (WiFi/Ethernet) ←→ ROS2 DDS Network ←→ Raspberry Pi Robot
```

**Multi-User**:
```
Desktop 1 ─┐
Desktop 2 ─┼─→ ROS2 Network ←→ Raspberry Pi Robot
Desktop 3 ─┘
```

### Hardware-Software Integration

#### GPIO Pin Mapping

**Motor Control** (16 pins):
- Motors 1-4: Each has IN1, IN2 (direction), PWM (speed)
- Controlled by: `motor_driver` node
- Command source: `/cmd_vel` topic

**Encoders** (8 pins):
- 4 encoders: Each has A, B channels (quadrature)
- Read by: `encoder_node` node
- Publishes to: `/odom` topic

**I2C Sensors** (I2C bus 1):
- MPU6050 IMU at address 0x68
- Read by: `imu_node` node
- Publishes to: `/imu/data` topic

### System Deployment Options

#### Option 1: Standalone Robot
```
Raspberry Pi (Self-contained)
  ├─ All ROS2 nodes running locally
  ├─ Optional: Local web interface
  └─ Autonomous operation mode
```

#### Option 2: Tethered Operation
```
Desktop Client (Development)
  ├─ ROS2 nodes for visualization
  ├─ Digital Twin simulation
  └─ Connected to:
      └─ Raspberry Pi Robot (Field operation)
          ├─ Motor control
          ├─ Sensors
          └─ Base controller
```

#### Option 3: Cloud Integration (Future)
```
Desktop/Mobile Clients
    ↓
Cloud Services (Azure/AWS)
  ├─ Data logging
  ├─ Fleet management
  └─ Remote commands
    ↓
Raspberry Pi Robots (Multiple)
```

### Configuration Management

#### Desktop Client Configuration
- **Build-time**: CMakeLists.txt (ROS2 enabled/disabled)
- **Run-time**: Command-line arguments, environment variables
- **UI**: Widget layouts saved/restored

#### Robot Configuration
- **Static**: `robot_config.yaml` - Robot parameters
- **Launch**: `robot.launch.py` - Node startup configuration
- **Run-time**: ROS2 parameters (can be updated dynamically)

### Error Handling & Recovery

#### Desktop Client
```cpp
try {
    ros2Interface->publishVelocity(x, z);
} catch (const std::exception& e) {
    Logger::instance().error("ROS2 publish failed");
    emit errorOccurred(QString::fromStdString(e.what()));
    // UI shows error, graceful degradation
}
```

#### Robot Nodes
```cpp
try {
    motor_driver_->setSpeed(left, right);
} catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "Motor control error");
    // Emergency stop, publish error status
}
```

### Performance Characteristics

| Component | Update Rate | Latency | Notes |
|-----------|-------------|---------|-------|
| Motor Control | 20 Hz | < 50ms | Velocity commands |
| IMU Sensor | 50 Hz | < 20ms | High-frequency inertial data |
| Encoder Odometry | 20 Hz | < 50ms | Position updates |
| Video Stream | 30 fps | 100-200ms | Depends on network |
| Digital Twin Sim | 50 Hz | N/A | Local simulation |
| UI Updates | 10-30 Hz | < 100ms | Visual updates |

### Security Considerations

#### Current Implementation
- Local network operation (LAN/WiFi)
- No authentication (development mode)
- ROS2 DDS security disabled

#### Production Recommendations
- Enable ROS2 DDS Security (SROS2)
- Network segmentation (robot VLAN)
- Certificate-based authentication
- Encrypted communication
- Access control lists

---

## Documentation Cross-References

### For Developers
- **Code Reference**: [CODE_REFERENCE.md](/CODE_REFERENCE.md) - API and class documentation
- **File Structure**: [FILE_STRUCTURE.md](FILE_STRUCTURE.md) - Project organization
- **Changelog**: [CHANGELOG.md](/CHANGELOG.md) - Version history

### For Users
- **Quick Start**: [QUICKSTART.md](../QUICKSTART.md) - Getting started guide
- **Main README**: [README.md](../README.md) - Overview and build instructions
- **ROS2 Workspace**: [raspberry-pi/README.md](/raspberry-pi/README.md) - Robot setup

---

This architecture provides:

- ✅ Modularity
- ✅ Scalability
- ✅ Testability
- ✅ Maintainability
- ✅ Extensibility
- ✅ Cross-component integration
- ✅ Hardware-software abstraction
- ✅ Distributed operation support
