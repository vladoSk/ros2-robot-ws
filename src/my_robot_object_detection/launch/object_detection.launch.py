#!/usr/bin/env python3
"""
Launch file for object detection system
Starts object detector and optionally the follower node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('my_robot_object_detection')

    # Path to config file
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'object_detection_params.yaml'
    ])

    # Declare launch arguments
    enable_follower_arg = DeclareLaunchArgument(
        'enable_follower',
        default_value='true',
        description='Enable object follower node'
    )

    # Object detector node
    object_detector_node = Node(
        package='my_robot_object_detection',
        executable='object_detector',
        name='object_detector',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )

    # Object follower node
    object_follower_node = Node(
        package='my_robot_object_detection',
        executable='object_follower',
        name='object_follower',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(LaunchConfiguration('enable_follower')),
        emulate_tty=True
    )

    return LaunchDescription([
        enable_follower_arg,
        object_detector_node,
        object_follower_node,
    ])
