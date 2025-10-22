# 4-Wheel Differential Robot ROS2 Project

A complete ROS2 (Jazzy) project for a 4-wheel differential drive robot running on Raspberry Pi with L298N motor driver, IMU sensor, and encoders.

## Project Structure

```
ros2_robot_ws/
├── src/
│   ├── motor_control/          # L298N Motor Driver Control
│   │   ├── src/
│   │   │   └── motor_driver.cpp
│   │   ├── include/motor_control/
│   │   │   └── motor_driver.hpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── imu_sensor/             # IMU (MPU6050) Sensor Node
│   │   ├── src/
│   │   │   └── imu_node.cpp
│   │   ├── include/imu_sensor/
│   │   │   └── imu_node.hpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── encoder_odometry/       # Encoder & Odometry Node
│   │   ├── src/
│   │   │   └── encoder_node.cpp
│   │   ├── include/encoder_odometry/
│   │   │   └── encoder_node.hpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── robot_controller/       # Main Robot Controller
│       ├── src/
│       │   └── robot_controller.cpp
│       ├── include/robot_controller/
│       │   └── robot_controller.hpp
│       ├── CMakeLists.txt
│       └── package.xml
└── [build/, install/, log/]    # Generated during build
```

## Hardware Configuration

### Motor Setup (L298N Motor Driver)

**4 Motors in Differential Drive Configuration:**
- Motor 1 & 3: Left side (Front & Rear)
- Motor 2 & 4: Right side (Front & Rear)

### GPIO Pin Assignments

**Motor 1 (Front Left):**
- IN1: GPIO 17 (direction)
- IN2: GPIO 27 (direction)
- PWM: GPIO 22 (speed)

**Motor 2 (Front Right):**
- IN1: GPIO 23 (direction)
- IN2: GPIO 24 (direction)
- PWM: GPIO 25 (speed)

**Motor 3 (Rear Left):**
- IN1: GPIO 5 (direction)
- IN2: GPIO 6 (direction)
- PWM: GPIO 12 (speed)

**Motor 4 (Rear Right):**
- IN1: GPIO 13 (direction)
- IN2: GPIO 19 (direction)
- PWM: GPIO 26 (speed)

### Sensors

**IMU Sensor (MPU6050):**
- I2C Bus: 1 (/dev/i2c-1)
- I2C Address: 0x68 (default)
- Outputs: Acceleration (3-axis), Angular velocity (3-axis)

**Encoders (on Motors):**
- Encoder 1 (Motor 1): GPIO 4 (A), GPIO 14 (B)
- Encoder 2 (Motor 2): GPIO 15 (A), GPIO 18 (B)
- Encoder 3 (Motor 3): GPIO 2 (A), GPIO 3 (B)
- Encoder 4 (Motor 4): GPIO 7 (A), GPIO 8 (B)

## Installation & Setup

### Prerequisites

1. **Raspberry Pi OS** (Ubuntu 24.04 recommended for ROS2 Jazzy)
2. **ROS2 Jazzy** installed and sourced

### Installation Steps

1. **Source ROS2 Setup:**
   ```bash
   source /opt/ros/jazzy/setup.sh
   ```

2. **Build the Workspace:**
   ```bash
   cd ~/ros2_robot_ws
   colcon build
   ```

3. **Source the Workspace:**
   ```bash
   source install/setup.sh
   ```

### Optional: Build Individual Package

To build a specific package:
```bash
colcon build --packages-select motor_control
```

## ROS2 Nodes

### 1. Motor Driver Node (`motor_driver`)
- **Package:** `motor_control`
- **Executable:** `motor_driver`
- **Subscribes to:** `/cmd_vel` (geometry_msgs/Twist)
- **Parameters:**
  - `wheel_base` (0.2 m) - Distance between left/right wheels
  - `wheel_radius` (0.05 m) - Wheel radius
  - `max_speed` (1.0 m/s) - Maximum linear speed

**Description:** Converts velocity commands from `/cmd_vel` to motor control signals using differential drive kinematics. Each motor receives individual PWM speed and direction control.

### 2. IMU Node (`imu_node`)
- **Package:** `imu_sensor`
- **Executable:** `imu_node`
- **Publishes to:** `/imu/data` (sensor_msgs/Imu)
- **Parameters:**
  - `update_rate` (50.0 Hz) - Sensor update frequency
  - `i2c_bus` (1) - I2C bus number
  - `i2c_address` (0x68) - MPU6050 address

**Description:** Reads accelerometer and gyroscope data from MPU6050 sensor via I2C. Publishes standardized IMU messages with proper covariance matrices.

### 3. Encoder/Odometry Node (`encoder_node`)
- **Package:** `encoder_odometry`
- **Executable:** `encoder_node`
- **Publishes to:** `/odom` (nav_msgs/Odometry)
- **Parameters:**
  - `wheel_radius` (0.05 m) - Wheel radius
  - `wheel_base` (0.2 m) - Distance between wheels
  - `counts_per_rev` (20) - Encoder counts per revolution
  - `update_rate` (20.0 Hz) - Odometry update frequency

**Description:** Reads encoder counts from GPIO pins and calculates robot odometry (position and orientation) using differential drive kinematics.

### 4. Robot Controller Node (`robot_controller`)
- **Package:** `robot_controller`
- **Executable:** `robot_controller`
- **Subscribes to:**
  - `/cmd_vel` (geometry_msgs/Twist) - Command velocities
  - `/imu/data` (sensor_msgs/Imu) - IMU measurements
  - `/odom` (nav_msgs/Odometry) - Odometry data
- **Publishes to:**
  - `/cmd_vel_filtered` (geometry_msgs/Twist) - Filtered motor commands
  - `/robot_status` (std_msgs/String) - Robot status
- **Parameters:**
  - `control_loop_rate` (20.0 Hz) - Main control loop frequency
  - `max_linear_velocity` (1.0 m/s) - Maximum linear speed
  - `max_angular_velocity` (2.0 rad/s) - Maximum angular speed
  - `min_battery_voltage` (7.0 V) - Minimum safe battery voltage

**Description:** Coordinates all subsystems, validates commands, monitors battery, and manages emergency stops.

## Running the Nodes

### Terminal 1 - Core Motor Control System
```bash
source install/setup.sh
ros2 run motor_control motor_driver
```

### Terminal 2 - IMU Sensor Node
```bash
source install/setup.sh
ros2 run imu_sensor imu_node
```

### Terminal 3 - Encoder Odometry Node
```bash
source install/setup.sh
ros2 run encoder_odometry encoder_node
```

### Terminal 4 - Robot Controller
```bash
source install/setup.sh
ros2 run robot_controller robot_controller
```

### Terminal 5 - Send Test Commands
```bash
source install/setup.sh

# Move forward
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Rotate in place
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# Move with rotation
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'
```

## Key Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Input | Command velocities |
| `/cmd_vel_filtered` | geometry_msgs/Twist | Output | Filtered motor commands |
| `/imu/data` | sensor_msgs/Imu | Output | IMU sensor data |
| `/odom` | nav_msgs/Odometry | Output | Robot odometry |
| `/robot_status` | std_msgs/String | Output | Robot status info |

## Differential Drive Kinematics

For a 4-wheel differential robot, the motor speeds are calculated as:

$$v_{left} = v_x - \frac{v_z \cdot L}{2}$$

$$v_{right} = v_x + \frac{v_z \cdot L}{2}$$

Where:
- $v_x$ = linear velocity (m/s)
- $v_z$ = angular velocity (rad/s)
- $L$ = wheel base (distance between left/right wheels)

## Customization Guide

### Change Motor Pin Assignments
Edit the pin constants in `motor_control/include/motor_control/motor_driver.hpp`:
```cpp
static const int MOTOR1_PIN1 = 17;  // Change to your GPIO pin
static const int MOTOR1_PWM_PIN = 22;
```

### Change Sensor Update Rates
Pass ROS2 parameters when launching:
```bash
ros2 run imu_sensor imu_node --ros-args -p update_rate:=100.0
ros2 run encoder_odometry encoder_node --ros-args -p update_rate:=30.0
```

### Adjust Robot Parameters
Modify parameters in launch or via command line:
```bash
ros2 run motor_control motor_driver --ros-args \
  -p wheel_base:=0.25 \
  -p wheel_radius:=0.06
```

## Hardware Implementation Notes

### GPIO Control Implementation
The current code includes placeholder functions. For actual GPIO control, you need to:

1. **Install GPIO Library** (WiringPi or gpiozero):
   ```bash
   pip install gpiozero pigpio
   sudo apt install wiringpi
   ```

2. **Implement GPIO Functions** in `motor_driver.cpp`:
   ```cpp
   #include <wiringPi.h>
   // or
   #include <gpiozero/gpiozero.h>
   ```

### I2C Communication
For IMU sensor I2C communication:
```bash
sudo apt install libi2c-dev i2c-tools
pip install smbus2
```

### Running as Real-Time Process
For better performance:
```bash
sudo chrt -rr 50 ros2 run motor_control motor_driver
```

## Building for Raspberry Pi ARM64

```bash
# Cross-compile or build natively on Pi
colcon build --parallel-workers 1  # Pi might run out of memory with multiple workers
```

## Troubleshooting

### Nodes won't start
- Check ROS2 is sourced: `echo $ROS_DISTRO`
- Rebuild workspace: `colcon build --symlink-install`

### Motor not responding
- Verify GPIO pins in code match your wiring
- Check L298N power supply (usually 12V)
- Test GPIO with `raspi-gpio`

### IMU not detected
- Check I2C device: `i2cdetect -y 1`
- Verify MPU6050 at address 0x68
- Check I2C pull-up resistors

### Encoders not reading
- Verify encoder connections
- Check GPIO input configuration
- Use `gpioreadall` to monitor pins

## Next Steps

1. **Implement GPIO Libraries** - Replace placeholder GPIO functions with actual hardware control
2. **Add Launch Files** - Create launch files to start all nodes with one command
3. **Add Configuration Files** - Create YAML files for robot parameters
4. **Add Transforms** - Implement TF2 for coordinate frame management
5. **Add Navigation Stack** - Integrate with ROS2 Nav2 for autonomous navigation
6. **Add Safety Features** - Implement timeout watchdogs and obstacle detection

## License & Maintenance

This project is part of the Precision Farming Robot 2.0.
Modify and expand as needed for your specific robot platform.

## Resources

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [ROS2 Concepts](https://docs.ros.org/en/jazzy/Concepts/Basic.html)
- [Raspberry Pi GPIO](https://www.raspberrypi.org/documentation/computers/os.html#gpio-and-the-40-pin-header)
- [L298N Documentation](https://www.electronicoscoyotitos.com/wp-content/uploads/2017/08/L298N-ESP-datasheet.pdf)
- [MPU6050 Documentation](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
