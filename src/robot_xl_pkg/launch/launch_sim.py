import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. DEFINE PATHS (ADJUST THESE)
    # Assume your YAML file is in a 'config' directory in a package named 'your_robot_pkg'
    pkg_share_dir = get_package_share_directory('robot_xl_pkg')
    controller_config_path = os.path.join(pkg_share_dir, 'config', 'diff_drive_controller.yaml')
    
    # 2. LAUNCH GAZEBO (with your robot model)
    # (Insert your current Gazebo launch method here, e.g., using ExecuteProcess or the Gazebo-ROS bridge)
    # ...

    # 3. START THE CONTROLLER MANAGER (loading the YAML parameters)
    # This node loads the parameters which the gazebo_ros2_control plugin will read.
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config_path], # <-- THIS LOADS YOUR YAML FILE
        output='screen'
    )
    
    # 4. START ROBOT STATE PUBLISHER (if not already running)
    robot_description_path = os.path.join(get_package_share_directory('my_robot_description'), 'urdf', 'robot.urdf.xacro')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', robot_description_path])}],
        output='screen'
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        # ... Gazebo launch ...
        controller_manager_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
    ])