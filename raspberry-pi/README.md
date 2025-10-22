# Raspberry Pi Code

This directory contains the ROS2 Jazzy project for the Precision Farming Robot 2.0's Raspberry Pi controller.

## 🤖 Robot Overview

A complete **4-wheel differential drive robot** system with:
- **Motor Control**: L298N driver controlling 4 DC motors with encoders
- **Sensors**: MPU6050 IMU for acceleration/rotation measurement
- **Odometry**: Encoder-based position and orientation tracking
- **Controller**: Centralized robot controller coordinating all subsystems

## 📦 What's Included

### ROS2 Packages (in `ros2_robot_ws/src/`)
1. **motor_control** - L298N motor driver control node
2. **imu_sensor** - MPU6050 IMU sensor reading node
3. **encoder_odometry** - Encoder and odometry calculation node
4. **robot_controller** - Main robot control coordinator

### Documentation
- **README.md** - Full technical documentation (470+ lines)
- **QUICKSTART.md** - Quick start and testing guide (350+ lines)
- **PROJECT_OVERVIEW.md** - Project structure and reference

### Configuration & Scripts
- **launch/robot.launch.py** - Launch all nodes together
- **config/robot_config.yaml** - Tunable robot parameters
- **setup.sh** - Build and setup script
- **gpio_helper_examples.py** - GPIO implementation examples

## ⚡ Quick Start

### 1. Build the Project
```bash
cd ros2_robot_ws
./setup.sh
```

### 2. Source the Workspace
```bash
source ros2_robot_ws/install/setup.sh
```

### 3. Launch the Robot
```bash
ros2 launch robot robot.launch.py
```

### 4. Send Commands (in another terminal)
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0}}'

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0}, angular: {z: 1.0}}'
```

## 🔧 Hardware Setup

### GPIO Pin Configuration
- **Motors**: 12 GPIO pins for 4 motors (direction + PWM)
- **Encoders**: 8 GPIO pins for quadrature encoder inputs
- **IMU**: I2C Bus 1 (SCL/SDA pins)

See `ros2_robot_ws/README.md` for complete pin mapping.

## 📚 Documentation

| Document | Content |
|----------|---------|
| [`ros2_robot_ws/README.md`](ros2_robot_ws/README.md) | Full technical documentation with API reference |
| [`ros2_robot_ws/QUICKSTART.md`](ros2_robot_ws/QUICKSTART.md) | Quick start guide and common commands |
| [`ros2_robot_ws/PROJECT_OVERVIEW.md`](ros2_robot_ws/PROJECT_OVERVIEW.md) | Project structure and architecture |

## 🎯 Key Features

✅ **4 ROS2 C++ Nodes** (~600 lines of code)
- Motor driver with differential kinematics
- IMU sensor data publisher
- Encoder-based odometry calculation
- Main robot controller

✅ **Differential Drive Kinematics**
- Proper velocity calculations for 4-motor differential system
- Smooth rotation and forward motion coordination

✅ **Safety Features**
- Emergency stop functionality
- Velocity limiting
- Battery voltage monitoring
- Command timeout

✅ **Clean, Understandable Code**
- Well-documented C++
- Standard ROS2 patterns
- Easy to modify and extend

✅ **Minimal but Complete**
- No unnecessary complexity
- Perfect for learning
- Production-ready structure

## 🛠️ Development

### Build Individual Package
```bash
colcon build --packages-select motor_control
```

### Enable Debug Logging
```bash
ros2 run motor_control motor_driver --ros-args --log-level DEBUG
```

### Monitor Topics
```bash
ros2 topic list           # View all topics
ros2 topic echo /imu/data # Monitor IMU data
```

## 📋 Requirements

- ROS2 Jazzy (Ubuntu 24.04 or compatible)
- C++17 or later
- CMake 3.8+
- Raspberry Pi (4B recommended)

## 🔌 Hardware Required

- Raspberry Pi (4B or better)
- L298N Motor Driver
- 4x DC Motors with encoders
- MPU6050 IMU Sensor
- 12V Power Supply (motors)
- 5V Power Supply (Pi)

## 📖 Learning Path

1. **Start Here**: [`QUICKSTART.md`](ros2_robot_ws/QUICKSTART.md) - Get it running in 5 minutes
2. **Then Read**: [`README.md`](ros2_robot_ws/README.md) - Understand the architecture
3. **Reference**: [`PROJECT_OVERVIEW.md`](ros2_robot_ws/PROJECT_OVERVIEW.md) - Deep dive

## 🚀 Next Steps

1. Implement GPIO library integration (WiringPi/gpiozero)
2. Test with actual hardware
3. Add RViz visualization
4. Integrate Nav2 for autonomous navigation

## 📝 Notes

- All code uses C++17 standard
- Follows ROS2 naming conventions
- Includes proper error handling
- GPIO functions are placeholders (implement with your chosen GPIO library)

## 📞 Support

For detailed information:
- Read the [`README.md`](ros2_robot_ws/README.md) for full documentation
- Check [`QUICKSTART.md`](ros2_robot_ws/QUICKSTART.md) for common tasks
- See `gpio_helper_examples.py` for GPIO implementation examples

---

**Status**: ✅ Ready for development and testing
**Last Updated**: October 22, 2025
