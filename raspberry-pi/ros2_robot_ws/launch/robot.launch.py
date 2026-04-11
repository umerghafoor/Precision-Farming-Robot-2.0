#!/usr/bin/env python3
"""
Launch file for the 4-wheel differential robot
Starts all necessary nodes: motor driver, IMU, encoders, and robot controller
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Starting 4-Wheel Differential Robot..."),
        
        # SPI Controller Bridge Node
        Node(
            package='motor_control',
            executable='spi_controller_bridge',
            name='spi_controller_bridge',
            output='screen',
            parameters=[
                {'cmd_vel_topic': '/cmd_vel'},
                {'servo1_topic': '/servo1/angle'},
                {'servo2_topic': '/servo2/angle'},
                {'spi_device': '/dev/spidev0.0'},
                {'spi_mode': 0},
                {'spi_bits_per_word': 8},
                {'spi_speed_hz': 500000},
                {'wheel_base': 0.2},
                {'max_linear_velocity': 1.0},
                {'cmd_timeout_sec': 0.5},
                {'tx_rate_hz': 20.0},
                {'default_servo_angle': 90},
            ],
            emulate_tty=True,
        ),
        
        # IMU Sensor Node
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

        # Camera Sensor Node
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
        
        # Encoder Odometry Node
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
        
        # Robot Controller Node
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
        
        LogInfo(msg="All nodes started successfully!"),
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
