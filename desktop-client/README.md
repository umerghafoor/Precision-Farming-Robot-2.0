# Precision Farming Robot - Desktop Client

A professional, modular Qt6-based desktop application for controlling and monitoring the Precision Farming Robot with ROS2 integration and Digital Twin simulation capabilities.

## 🏗️ Architecture Overview

This application follows a **clean, modular architecture** with clear separation of concerns:

```
┌─────────────────────────────────────────────────────────────┐
│                      Application Layer                      │
│  (Orchestrates all subsystems - Facade Pattern)             │
└───────────────┬─────────────────┬──────────────┬────────────┘
                │                 │              │
        ┌───────▼──────┐  ┌───────▼──────┐  ┌──▼─────────┐
        │  ROS2 Module │  │ Digital Twin │  │ UI System  │
        │              │  │   Module     │  │            │
        └──────────────┘  └──────────────┘  └────────────┘
```

### Key Design Patterns

1. **Facade Pattern** - Application class provides unified interface
2. **Factory Pattern** - WidgetManager creates widgets dynamically
3. **Observer Pattern** - Qt signals/slots for event handling
4. **Strategy Pattern** - Digital Twin modes (Synchronized/Simulated/Offline)
5. **Singleton Pattern** - Logger utility

## 📁 Project Structure

```
desktop-client/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # ROS2 package manifest
├── README.md                   # This file
│
├── src/
│   ├── main.cpp               # Application entry point
│   │
│   ├── core/                  # Core application framework
│   │   ├── Application.h/cpp  # Main application orchestrator
│   │   └── WidgetManager.h/cpp # Widget factory & registry
│   │
│   ├── ros2/                  # ROS2 Integration (isolated)
│   │   └── ROS2Interface.h/cpp # ROS2 communication layer
│   │
│   ├── twin/                  # Digital Twin Module (isolated)
│   │   ├── DigitalTwin.h/cpp  # Main twin controller
│   │   ├── TwinState.h/cpp    # State management
│   │   └── TwinSimulator.h/cpp # Physics simulation
│   │
│   ├── ui/                    # User Interface
│   │   ├── MainWindow.h/cpp   # Main application window
│   │   └── widgets/           # Modular widget system
│   │       ├── BaseWidget.h/cpp              # Widget base class
│   │       ├── VideoStreamWidget.h/cpp       # Camera feeds
│   │       ├── CommandControlWidget.h/cpp    # Robot control
│   │       ├── SensorDataWidget.h/cpp        # Sensor display
│   │       └── TwinVisualizationWidget.h/cpp # Digital twin
│   │
│   └── utils/                 # Utilities
│       └── Logger.h/cpp       # Logging system
│
└── resources/                 # Resources (future)
    └── ui/
```

## 🎯 Key Features

### 1. **Modular Widget System**
- **Fully dockable** widgets using Qt's QDockWidget
- **Add/remove widgets** dynamically at runtime
- **Rearrangeable** workspace - drag and drop to organize
- **Four core widgets:**
  - 📹 Video Stream Widget
  - 🎮 Command & Control Widget
  - 📊 Sensor Data Widget
  - 🤖 Digital Twin Visualization Widget

### 2. **ROS2 Integration**
- **Threaded ROS2** interface (non-blocking)
- **Publishers:** Velocity commands, robot commands
- **Subscribers:** Camera images, IMU data, status updates
- **Seamless Qt integration** via signals/slots

### 3. **Digital Twin**
- **Three operating modes:**
  - **Synchronized:** Real-time sync with physical robot
  - **Simulated:** Run physics simulation independently
  - **Offline:** No connection
- **State management:** Position, velocity, sensors, battery
- **Physics simulation:** Basic kinematics and sensor modeling

### 4. **Scalability Features**
- **Clean separation** between ROS2, Twin, and UI
- **Easy to extend** with new widgets
- **Plugin-ready** architecture
- **SOLID principles** throughout

## 🛠️ Building the Application

### Prerequisites

```bash
# Install Qt6
sudo apt install qt6-base-dev qt6-multimedia-dev

# Install ROS2 (Humble/Iron/Jazzy)
# Follow: https://docs.ros.org/

# Install dependencies
sudo apt install build-essential cmake
```

### Build Instructions

```bash
# Source ROS2
source /opt/ros/<distro>/setup.bash

# Navigate to workspace
cd /path/to/Precision-Farming-Robot-2.0/desktop-client

# Build with colcon (ROS2 way)
colcon build

# Or build with CMake directly
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Run the Application

```bash
# Source the workspace
source install/setup.bash

# Run
./build/PrecisionFarmingDesktopClient

# Or if built with colcon
ros2 run precision_farming_desktop_client PrecisionFarmingDesktopClient
```

## 🎮 Usage Guide

### Starting the Application

1. **Launch the client**
2. **Add widgets** via Menu: Widgets → Add [Widget Type]
3. **Connect to ROS2**: ROS2 → Connect
4. **Start simulation** (optional): Simulation → Start Simulation

### Widget Operations

- **Drag & drop** widget titles to rearrange
- **Float widgets** by dragging them out
- **Close widgets** by clicking the X on the widget title
- **Add multiple** instances of the same widget type

### Command & Control

- Use **sliders** to control robot velocity
- **STOP button** - Normal stop
- **EMERGENCY STOP** - Immediate halt + ROS2 command

### Digital Twin Modes

- **Synchronized**: Mirrors real robot state from ROS2
- **Simulated**: Runs independent simulation
- **Offline**: Disconnected state

## 🔧 Extending the Application

### Adding a New Widget

1. **Create widget class** inheriting from `BaseWidget`:

```cpp
// MyCustomWidget.h
#include "BaseWidget.h"

class MyCustomWidget : public BaseWidget {
    Q_OBJECT
public:
    explicit MyCustomWidget(QWidget *parent = nullptr);
    bool initialize() override;
    QString displayName() const override { return "My Widget"; }
private:
    void setupUI();
};
```

2. **Register in WidgetManager**:

```cpp
// In WidgetManager.h - add enum
enum class WidgetType {
    // ... existing types
    MyCustomWidget
};

// In WidgetManager.cpp - add to factory
case WidgetType::MyCustomWidget:
    widget = new MyCustomWidget(parent);
    break;
```

3. **Add to CMakeLists.txt**:

```cmake
set(SOURCES
    # ... existing sources
    src/ui/widgets/MyCustomWidget.cpp
)
```

### Adding ROS2 Topics

Edit `src/ros2/ROS2Interface.cpp`:

```cpp
// Add subscriber
m_mySubscriber = m_node->create_subscription<MyMsgType>(
    "/my_topic", 10,
    std::bind(&ROS2Interface::myCallback, this, std::placeholders::_1));
```

## 📊 System Requirements

- **OS:** Linux (Ubuntu 20.04/22.04 recommended)
- **Qt:** 6.2 or higher
- **ROS2:** Humble/Iron/Jazzy
- **C++:** C++17 or higher
- **RAM:** 4GB minimum, 8GB recommended
- **CPU:** Multi-core recommended for simulation

## 🔍 Debugging

### Enable Verbose Logging

```cpp
// In main.cpp
Logger::instance().setLogLevel(Logger::Level::Debug);
```

### Check Log File

```bash
tail -f PrecisionFarmingClient.log
```

## 📝 Code Quality

- ✅ **RAII** principles for resource management
- ✅ **Smart pointers** (std::unique_ptr) for ownership
- ✅ **Const correctness** throughout
- ✅ **Qt signals/slots** for loose coupling
- ✅ **Comprehensive logging**
- ✅ **Thread-safe** ROS2 integration

## 🚀 Future Enhancements

- [ ] 3D visualization using Qt3D or OpenGL
- [ ] Plugin system for third-party widgets
- [ ] Configuration persistence (save/load layouts)
- [ ] Video recording functionality
- [ ] Data logging and playback
- [ ] Map visualization
- [ ] Path planning interface
- [ ] Multi-robot support

## 📄 License

MIT License - See LICENSE file for details

## 🤝 Contributing

1. Follow existing code style
2. Add documentation for new features
3. Test with both ROS2 and simulation modes
4. Update README for significant changes

## 📞 Support

For issues and questions, please create an issue on the GitHub repository.

---

**Built with ❤️ for Precision Agriculture**
- Mission planning and analysis

## Setup

Instructions for setting up the desktop client will be added here.

## Usage

Usage instructions will be added here.
