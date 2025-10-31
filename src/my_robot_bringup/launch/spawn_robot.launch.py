import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, RegisterEventHandler

def generate_launch_description():
    # Get controller config file path
    controller_config_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'config',
        'my_robot_controllers.yaml'
    )
    # Get URDF file path
    urdf_file_name = 'my_robot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        urdf_file_name
    )
    # Gazebo launch
    gazebo_launch_file_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    gazebo_world_path = os.path.join(get_package_share_directory('my_robot_bringup'), 'worlds', 'lawn.sdf')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_file_dir, 'gazebo.launch.py')),
        launch_arguments={
            'world': gazebo_world_path,
        }.items(),
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
    )

    # Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config_path],
        output='screen',
    )

    # This RegisterEventHandler will wait for the spawn_entity process to exit and then
    # start the controller_manager and the spawners in a sequence.
    # This is a more robust way to ensure all services are available before they are called.
    delayed_controller_manager_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                controller_manager_node,
                # The following spawners will be executed sequentially
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=controller_manager_node,
                        on_exit=[
                            Node(
                                package='controller_manager',
                                executable='spawner',
                                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                                output='screen',
                            ),
                        ],
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        gazebo_cmd,
        robot_state_publisher_node,
        spawn_entity,
        delayed_controller_manager_spawner,
    ])
