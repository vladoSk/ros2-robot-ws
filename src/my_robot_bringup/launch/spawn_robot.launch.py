import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Get URDF file path
    xacro_file_name = 'my_robot.urdf.xacro'
    xacro_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        xacro_file_name
    )
    # Gazebo launch
    gazebo_launch_file_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    gazebo_world_path = os.path.join(get_package_share_directory('my_robot_bringup'), 'worlds', 'lawn.sdf')

    # Controller manager YAML file
    controller_manager_config = os.path.join(
        get_package_share_directory('my_robot_description'),
        'config',
        'my_robot_controllers.yaml'
    )

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_file_dir, 'gazebo.launch.py')),
        launch_arguments={
            'world': gazebo_world_path,
            'force_system': 'false',  # Disable problematic force_system plugin
        }.items(),
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro ', xacro_path, ' controller_config_file:=', controller_manager_config])},
        ]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
    )

    # The 'spawner' node is a ROS 2 tool that calls the 'load_controller' service
    # on the controller_manager. The controller_manager is running inside Gazebo now,
    # thanks to the 'gazebo_ros2_control' plugin in the URDF.
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Twist stamper node to convert Twist to TwistStamped
    twist_stamper = Node(
        package='my_robot_bringup',
        executable='twist_stamper.py',
        name='twist_stamper',
        output='screen',
    )

    return LaunchDescription([
        gazebo_cmd,
        robot_state_publisher_node,
        spawn_entity,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        twist_stamper,
    ])
