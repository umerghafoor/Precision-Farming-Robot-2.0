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
        
        # Motor Driver Node
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
