# Quick Start Guide: 4-Wheel Differential Robot

## Installation (One-Time Setup)

### 1. Prerequisites
Make sure you have:
- ROS2 Jazzy installed on Raspberry Pi
- Source ROS2 setup in .bashrc:
  ```bash
  echo "source /opt/ros/jazzy/setup.sh" >> ~/.bashrc
  source ~/.bashrc
  ```

### 2. Build the Workspace
```bash
cd ~/ros2_robot_ws
./setup.sh
```

Or with dependency installation:
```bash
./setup.sh --install-deps --build
```

### 3. Source the Install
```bash
source ~/ros2_robot_ws/install/setup.sh
```

For automatic sourcing, add to ~/.bashrc:
```bash
echo "source ~/ros2_robot_ws/install/setup.sh" >> ~/.bashrc
source ~/.bashrc
```

## Running the Robot

### Option 1: Launch All Nodes (Recommended)
```bash
ros2 launch robot robot.launch.py
```

### Option 2: Run Individual Nodes in Separate Terminals

**Terminal 1 - Motor Driver:**
```bash
ros2 run motor_control motor_driver
```

**Terminal 2 - IMU Sensor:**
```bash
ros2 run imu_sensor imu_node
```

**Terminal 3 - Encoder Odometry:**
```bash
ros2 run encoder_odometry encoder_node
```

**Terminal 4 - Robot Controller:**
```bash
ros2 run robot_controller robot_controller
```

## Testing Commands

### Monitor Published Topics
```bash
# View all active topics
ros2 topic list

# Monitor motor commands
ros2 topic echo /cmd_vel

# Monitor IMU data
ros2 topic echo /imu/data

# Monitor odometry
ros2 topic echo /odom

# Monitor robot status
ros2 topic echo /robot_status
```

### Send Motion Commands
Open a new terminal and try these commands:

```bash
# 1. Move forward slowly (0.5 m/s)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 2. Move forward faster (1.0 m/s)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 3. Move backward (negative linear velocity)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 4. Rotate left (positive angular velocity)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}'

# 5. Rotate right (negative angular velocity)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}'

# 6. Move and rotate simultaneously (arc motion)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# 7. Stop (zero velocities)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## Node Architecture

```
                    ┌─────────────────────────────────┐
                    │   User/External Controller      │
                    │   (ROS2 node or remote input)   │
                    └────────────┬────────────────────┘
                                 │
                                 │ /cmd_vel (Twist)
                                 ▼
                    ┌─────────────────────────────────┐
                    │   Robot Controller Node         │
                    │ - Validates commands            │
                    │ - Monitors sensors              │
                    │ - Manages emergency stop        │
                    └────────┬────────────┬──────────┘
                             │            │
          /cmd_vel_filtered  │            │ /imu/data, /odom
                    ┌────────▼──┐  ┌─────▼────┐
                    │   Motor   │  │ Monitors │
                    │  Driver   │  │ (logging)│
                    └────────┬──┘  └──────────┘
                             │
                    ┌────────▼──────────────┐
                    │ GPIO PWM Signals     │
                    │ to L298N Motor Driver│
                    └────────┬──────────────┘
                             │
                    ┌────────▼──────────────┐
                    │  4 DC Motors         │
                    │  (differential drive)│
                    └──────────────────────┘

    ┌─────────────────────────────────────────────────────────┐
    │              Sensor Feedback Subsystem                  │
    │                                                          │
    │  ┌─────────────────────────────────────────────────┐   │
    │  │ GPIO Pins ◄── Encoders (each motor)             │   │
    │  │    ↓                                              │   │
    │  │ Encoder Node ► /odom (Odometry)                  │   │
    │  │    ↓                                              │   │
    │  │ I2C Bus ◄── MPU6050 (Accelerometer + Gyro)       │   │
    │  │    ↓                                              │   │
    │  │ IMU Node ► /imu/data (Sensor data)               │   │
    │  └─────────────────────────────────────────────────┘   │
    │             (Feeds back to Robot Controller)            │
    └─────────────────────────────────────────────────────────┘
```

## Debugging Tips

### Check Node Status
```bash
ros2 node list           # List all running nodes
ros2 node info <node>    # Get info about specific node
```

### Check Topic Status
```bash
ros2 topic info <topic>  # Get info about a topic
ros2 topic hz <topic>    # Check publishing rate
```

### View Parameter Values
```bash
ros2 param list          # List all parameters
ros2 param get <node> <param>  # Get parameter value
ros2 param set <node> <param> <value>  # Set parameter value
```

### Enable Verbose Logging
```bash
export ROS_LOG_DIR=/tmp/ros_logs
ros2 run motor_control motor_driver --ros-args --log-level DEBUG
```

### Record and Playback Data
```bash
# Record all topics to bag file
ros2 bag record -a

# Playback recorded data
ros2 bag play rosbag2_*
```

## Common Issues

### "Node not found" error
- Make sure workspace is sourced: `source install/setup.sh`
- Rebuild if code was changed: `colcon build`

### Motors not responding
- Check GPIO pins in code match your wiring
- Verify L298N power supply connection
- Test GPIO with: `raspi-gpio get [pin_number]`

### IMU not detected
- Check I2C connection: `i2cdetect -y 1`
- Verify MPU6050 at address 0x68
- Check I2C pull-up resistors (should be 10kΩ)

### High CPU usage
- Reduce update rates in parameters
- Check for infinite loops or deadlocks
- Monitor with `top` or `htop`

## Performance Tuning

### Control Loop Rate
- Increase for faster response: 30-50 Hz (more CPU)
- Decrease for lower CPU: 10-20 Hz (slower response)

### Motor Speed Ramp
- Smooth acceleration/deceleration by implementing ramping
- Prevents mechanical stress on motors

### Sensor Filtering
- Add complementary filter for IMU fusion
- Apply moving average to encoders

## Next Advanced Features

1. **RViz Visualization** - Visualize robot state in 3D
2. **Nav2 Integration** - Autonomous navigation and mapping
3. **Safety Layer** - Obstacle detection and avoidance
4. **State Estimation** - Kalman filter for sensor fusion
5. **Web Dashboard** - Remote monitoring and control

See README.md for detailed documentation!
