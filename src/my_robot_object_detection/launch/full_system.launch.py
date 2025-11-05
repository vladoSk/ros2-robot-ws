#!/usr/bin/env python3
"""
Full system launch file
Launches the robot in Gazebo with object detection
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    bringup_pkg = FindPackageShare('my_robot_bringup')
    detection_pkg = FindPackageShare('my_robot_object_detection')

    # Launch arguments
    enable_follower_arg = DeclareLaunchArgument(
        'enable_follower',
        default_value='false',
        description='Enable object follower (robot will move automatically)'
    )

    # Include robot spawn launch file
    robot_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_pkg,
                'launch',
                'spawn_robot.launch.py'
            ])
        )
    )

    # Include object detection launch file
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                detection_pkg,
                'launch',
                'object_detection.launch.py'
            ])
        ),
        launch_arguments={
            'enable_follower': LaunchConfiguration('enable_follower')
        }.items()
    )

    return LaunchDescription([
        enable_follower_arg,
        robot_spawn_launch,
        object_detection_launch,
    ])
