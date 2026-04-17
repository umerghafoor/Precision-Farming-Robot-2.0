#!/usr/bin/env python3
"""
Launch file for the 4-wheel differential robot
Starts all necessary nodes: motor driver, IMU, camera, encoders, and controller
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Starting 4-Wheel Differential Robot..."),

        Node(
            package='motor_control',
            executable='motor_driver',
            name='motor_driver',
            output='screen',
            parameters=[
                {'wheel_base': 0.2},
                {'wheel_radius': 0.05},
                {'max_speed': 1.0},
            ],
            emulate_tty=True,
        ),

        Node(
            package='imu_sensor',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[
                {'update_rate': 50.0},
                {'i2c_bus': 1},
                {'i2c_address': 0x68},
            ],
            emulate_tty=True,
        ),

        Node(
            package='camera_sensor',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 640},
                {'image_height': 480},
                {'publish_rate': 30.0},
                {'frame_id': 'camera_link'},
                {'camera_topic': '/camera/raw'},
            ],
            emulate_tty=True,
        ),

        Node(
            package='yolo_detection',
            executable='yolo_detection_node',
            name='yolo_detection_node',
            output='screen',
            parameters=[
                {'model_path': ''},  # Set model path in robot_config.yaml
                {'confidence_threshold': 0.25},
                {'iou_threshold': 0.45},
                {'input_size': 640},
                {'camera_topic': '/camera/raw'},
                {'annotated_topic': '/camera/detection'},
                {'results_topic': '/detections/results'},
                {'enable_visualization': True},
            ],
            emulate_tty=True,
        ),

        Node(
            package='encoder_odometry',
            executable='encoder_node',
            name='encoder_node',
            output='screen',
            parameters=[
                {'wheel_radius': 0.05},
                {'wheel_base': 0.2},
                {'counts_per_rev': 20},
                {'update_rate': 20.0},
            ],
            emulate_tty=True,
        ),

        Node(
            package='robot_controller',
            executable='robot_controller',
            name='robot_controller',
            output='screen',
            parameters=[
                {'control_loop_rate': 20.0},
                {'emergency_stop_enabled': True},
                {'max_linear_velocity': 1.0},
                {'max_angular_velocity': 2.0},
                {'min_battery_voltage': 7.0},
            ],
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
                {'image_rate_hz': 0.5},   # annotated detection frames → robot/image
                {'imu_rate_hz': 1.0},     # IMU heading/accel/gyro → robot/imu
                {'image_format': 'jpeg'}, # JPEG is smaller and faster over MQTT
                {'image_quality': 80},    # good quality/size balance
                {'camera_detection_transport': 'compressed'},  # /camera/detection is CompressedImage (YOLO output)
            ],
            emulate_tty=True,
        ),

        LogInfo(msg="All nodes started successfully!"),
    ])
