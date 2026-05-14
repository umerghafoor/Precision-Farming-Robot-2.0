#!/usr/bin/env python3
"""Launch the 4 core robot nodes."""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Starting robot nodes..."),

        Node(
            package='camera_sensor',
            executable='webcam_node',
            name='webcam_node',
            output='screen',
            parameters=[
                {'image_width': 640},
                {'image_height': 480},
                {'publish_rate': 10.0},
                {'frame_id': 'camera_link'},
                {'camera_topic': '/camera/raw'},
                {'color_jpeg_topic': '/camera/color_jpeg'},
                {'bit_depth': 4},
                {'jpeg_quality': 50},
            ],
            emulate_tty=True,
        ),

        Node(
            package='motor_control',
            executable='spi_controller_bridge',
            name='spi_controller_bridge',
            output='screen',
            parameters=[
                {'serial_port': 'auto'},
                {'cmd_vel_topic': '/cmd_vel'},
                {'servo1_topic': '/servo1/angle'},
                {'servo2_topic': '/servo2/angle'},
                {'wheel_base': 0.2},
                {'max_linear_velocity': 1.0},
                {'cmd_timeout_sec': 0.5},
                {'tx_rate_hz': 20.0},
                {'default_servo_angle': 90},
            ],
            emulate_tty=True,
        ),

        # IMU may not be connected; respawn so it comes up when plugged in
        Node(
            package='imu_sensor',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[
                {'serial_port': 'auto'},
                {'publish_rate_hz': 50.0},
            ],
            respawn=True,
            respawn_delay=5.0,
            emulate_tty=True,
        ),

        Node(
            package='mqtt_bridge',
            executable='mqtt_bridge_node',
            name='mqtt_bridge_node',
            output='screen',
            parameters=[
                {'mqtt_host': 'sanilinux.mullet-bull.ts.net'},
                {'mqtt_port': 1883},
                {'mqtt_keepalive': 60},
                {'odom_rate_hz': 1.0},
                {'image_rate_hz': 0.5},
                {'imu_rate_hz': 1.0},
                {'image_format': 'jpeg'},
                {'image_quality': 80},
            ],
            emulate_tty=True,
        ),

        LogInfo(msg="All nodes started."),
    ])
