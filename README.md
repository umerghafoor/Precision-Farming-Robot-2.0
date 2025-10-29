# Precision Farming Robot 2.0

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](./CHANGELOG.md)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Qt](https://img.shields.io/badge/Qt-6-green.svg)](https://www.qt.io/)

An advanced autonomous farming robot system designed for precision agriculture applications with real-time control, digital twin simulation, and comprehensive sensor integration.

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Project Components](#-project-components)
- [Quick Start](#-quick-start)
- [Documentation](#-documentation)
- [Hardware Requirements](#-hardware-requirements)
- [Development](#-development)
- [Contributing](#-contributing)
- [License](#-license)
- [Support](#-support)

---

## 🌟 Overview

The Precision Farming Robot 2.0 is a comprehensive robotics platform that combines hardware, firmware, software, and mobile/desktop applications to enable autonomous farming operations. The system leverages ROS2 for advanced navigation and control, providing both remote operation and autonomous capabilities.

### What Makes It Special?

- **🎯 Production-Ready**: ~6,000+ lines of well-tested, documented code
- **🏗️ Modular Architecture**: Clean separation of concerns with industry-standard design patterns
- **🤖 Digital Twin**: Virtual representation for simulation, testing, and development
- **📡 Distributed System**: Desktop client can control robot locally or remotely
- **🔧 Extensible**: Plugin-ready architecture for easy customization
- **📊 Real-Time Monitoring**: Live sensor data visualization and robot status
- **🛡️ Safety First**: Emergency stop, velocity limiting, and battery monitoring

---

## ✨ Key Features

### Desktop Client Features
- **Modern Qt6 Interface**: Professional, dockable widget system
- **ROS2 Integration**: Thread-safe communication with robot
- **Digital Twin Simulation**: Three modes (Synchronized/Simulated/Offline)
- **Real-Time Visualization**: Camera feeds, sensor data, robot state
- **Manual Control**: Velocity commands and directional motion
- **Extensible Widgets**: Easy to add custom visualizations

### Robot Features
- **4-Wheel Differential Drive**: Precise motion control
- **Sensor Suite**: IMU (MPU6050), wheel encoders, camera support
- **Autonomous Navigation**: Path planning and obstacle avoidance (planned)
- **Odometry**: Real-time position tracking
- **Safety Systems**: Emergency stop, velocity limits, battery monitoring
- **Modular ROS2 Nodes**: Independent, reusable components

---

## 🏗️ System Architecture

```txt
┌─────────────────────────────────────────────────────────────┐
│                    User Interfaces Layer                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │Desktop Client│  │  Mobile App  │  │   Web UI     │      │
│  │  (Qt6/C++)  │  │   (Planned)  │  │  (Planned)   │      │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘      │
└─────────┼──────────────────┼──────────────────┼─────────────┘
          │                  │                  │
          └──────────────────┼──────────────────┘
                             │
                    ┌────────▼────────┐
                    │  ROS2 Network   │
                    │   (DDS Layer)   │
                    └────────┬────────┘
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
    ┌────▼────┐      ┌───────▼────────┐   ┌────▼────┐
    │ Digital │      │  Raspberry Pi  │   │External │
    │  Twin   │      │  Robot Brain   │   │Services │
    │  (Qt)   │      │  (ROS2 Jazzy)  │   │(Cloud)  │
    └─────────┘      └───────┬────────┘   └─────────┘
                             │
                ┌────────────┼────────────┐
                │            │            │
         ┌──────▼──────┐ ┌──▼───┐ ┌─────▼──────┐
         │   Motors    │ │Sensors│ │  Encoders  │
         │ (L298N x4)  │ │(IMU)  │ │   (x4)    │
         └─────────────┘ └───────┘ └────────────┘
```

For detailed architecture information, see [ARCHITECTURE.md](desktop-client/docs/ARCHITECTURE.md).

---

## 📁 Project Components

### 🖥️ [Desktop Client](./desktop-client/)
**Qt6 C++ Application** - Control center and digital twin

- Modern, modular UI with dockable widgets
- ROS2 integration for robot communication
- Digital twin with physics simulation
- Real-time sensor visualization
- Video streaming support
- ~3,850 lines of production code

**Quick Start**: [Desktop Client README](./desktop-client/README.md)

---

### 🤖 [Raspberry Pi Code](./raspberry-pi/)
**ROS2 Jazzy Workspace** - Robot brain and control

Contains 4 ROS2 packages:
- **motor_control**: L298N motor driver with differential drive kinematics
- **imu_sensor**: MPU6050 IMU sensor integration
- **encoder_odometry**: Wheel encoder-based odometry
- **robot_controller**: Main coordinator with safety features

**Quick Start**: [Raspberry Pi README](./raspberry-pi/README.md)

---

### 🔧 [Firmware](./firmware/)
**Microcontroller Code** - Low-level hardware control

- Real-time motor control
- Sensor interfacing
- Communication protocols

**Status**: Placeholder (see README for details)

---

### 📱 [Mobile App](./mobile-app/)
**Mobile Application** - Remote control and monitoring

- Remote operation interface
- Real-time data visualization
- Field data collection

**Status**: Planned (see README for details)

---

### 🚀 [ROS2 Packages](./ros2/)
**Additional ROS2 Components** - Navigation and autonomy

- Advanced navigation algorithms
- Path planning
- Sensor fusion
- Autonomous operation modes

**Status**: Placeholder (see README for details)

---

## Getting Started

Each component has its own README with specific setup and usage instructions. Please refer to the individual directories for detailed information.

### Prerequisites

- Raspberry Pi 4 or newer
- Compatible microcontroller for firmware
- ROS2 (Humble or newer recommended)
- Development tools for mobile/desktop applications

## Development

This project is under active development. Contributions and feedback are welcome.

## License

[License information to be added]

## Contact

[Contact information to be added]