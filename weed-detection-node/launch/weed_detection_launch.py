#!/usr/bin/env python3
"""
Launch file for weed detection node
Starts the ArUco processor and optionally the image viewer
"""

from launch import LaunchDescription
from launch import conditions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    start_viewer = DeclareLaunchArgument(
        'start_viewer',
        default_value='false',
        description='Whether to start the image viewer'
    )
    
    # ArUco processor node
    aruco_processor = Node(
        package='weed_detection_node',
        executable='aruco_processor',
        name='aruco_processor',
        output='screen',
        parameters=[],
        remappings=[]
    )
    
    # Image viewer node (optional)
    image_viewer = Node(
        package='weed_detection_node',
        executable='image_viewer',
        name='image_viewer',
        output='screen',
        parameters=[],
        condition=conditions.IfCondition(LaunchConfiguration('start_viewer'))
    )
    
    return LaunchDescription([
        start_viewer,
        aruco_processor,
        # Uncomment to auto-start viewer:
        # image_viewer,
    ])

