# ROS2 Topics and Commands (Precision Farming Desktop Client)

This document lists ROS2 topics used by the desktop client and common `ros2` CLI commands to inspect them.

## Topics used by the client

- `/cmd_vel` (geometry_msgs/msg/Twist)
  - Purpose: Velocity command published by control widgets (linear.x, linear.y, angular.z). For this client, differential-drive behavior uses `linear.x` and `angular.z`.
  - Publisher: Desktop client (CommandControlWidget, MotionControlWidget)
  - QoS: default (10)

- `/robot_command` (std_msgs/msg/String)
  - Purpose: Send higher-level string commands such as `EMERGENCY_STOP` or custom commands.
  - Publisher: Desktop client (CommandControlWidget)

- `/robot_status` (std_msgs/msg/String)
  - Purpose: Status or telemetry messages from the robot (string payload).
  - Subscriber: Desktop client (ROS2Interface)

- `/camera/image_raw` (sensor_msgs/msg/Image)
  - Purpose: Raw camera images for VideoStream widget.
  - Subscriber: Desktop client (ROS2Interface)

- `/imu/data` (sensor_msgs/msg/Imu)
  - Purpose: IMU data for sensor visualization.
  - Subscriber: Desktop client (ROS2Interface)

## Common ros2 CLI commands

Replace topic names as needed. These examples assume ROS2 is sourced and your ROS_DOMAIN_ID / RMW implementation are configured.

- List topics

```bash
ros2 topic list
```

- Show topic type and info

```bash
ros2 topic info /cmd_vel
ros2 topic type /cmd_vel
```

- Echo messages on a topic

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /robot_status
```

- Publish a test Twist (one-shot)

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{ linear: { x: 0.5, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.1 } }" -1
```

- Publish a string command

```bash
ros2 topic pub /robot_command std_msgs/msg/String "{ data: 'EMERGENCY_STOP' }" -1
```

- Monitor topic bandwidth and rate (useful for image streams)

```bash
ros2 topic hz /camera/image_raw
ros2 topic bw /camera/image_raw
```

## Notes
- Topic names and message types are defined in `src/ros2/ROS2Interface.cpp` and the widget code under `src/ui/widgets`.
- If ROS2 is not available at build time, the client compiles in stub mode and will log publish actions instead of actually publishing messages.

If you want, I can add a README section with example launch files or a small test node to echo commands for quick testing.