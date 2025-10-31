# Creating a ROS2-controlled Robot in Gazebo

This guide outlines the steps to create a robot model described in URDF, include it in a Gazebo world, and enable ROS2 control with a camera sensor.

## 1. Prerequisites

Ensure you have ROS2 (we use Humble) and Gazebo 11 installed and sourced correctly.

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
```

## 2. Create a ROS2 Workspace

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
ros2 pkg create --build-type ament_cmake my_robot_description
ros2 pkg create --build-type ament_cmake my_robot_bringup
cd ..
colcon build
source install/setup.bash
```

## 3. Define the Robot Model (URDF)

Navigate to `~/robot_ws/src/my_robot_description/urdf` and create `my_robot.urdf`. This file will describe a brick-like robot with two front wheels, a rear support sphere with ball joint, and a top-mounted camera.

### `my_robot.urdf` (Simplified Structure)

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.25"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0.15 0.12 0" rpy="1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel -->
  <link name="right_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="0.15 -0.12 0" rpy="1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Caster Wheel (Sphere Support) -->
  <link name="caster_wheel_link">
    <visual>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel_link"/>
    <origin xyz="-0.15 0 -0.05"/>
  </joint>

  <!-- Camera Link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0 0 0.1"/>
  </joint>

  <!-- Gazebo specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  <gazebo reference="left_wheel_link">
    <material>Gazebo/Black</material>
  </gazebo>
  <gazebo reference="right_wheel_link">
    <material>Gazebo/Black</material>
  </gazebo>
  <gazebo reference="caster_wheel_link">
    <material>Gazebo/Grey</material>
  </gazebo>
  <gazebo reference="camera_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <!-- ROS2 Control Plugin -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_robot_description)/config/my_robot_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Differential Drive Controller -->
  <gazebo>
    <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <ros> <!-- ROS 2 specific parameters -->
        <namespace>/my_robot</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.24</wheel_separation>
      <wheel_radius>0.05</wheel_radius>
      <publish_odom>true</publish_odom>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_wheel_joint_state>true</publish_wheel_joint_state>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

  <!-- Camera Sensor Plugin -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
      <update_rate>30.0</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/my_robot</namespace>
          <argument>--ros-args -r image_raw:=camera/image_raw</argument>
          <argument>--ros-args -r camera_info:=camera/camera_info</argument>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### `my_robot_controllers.yaml`

Create this file in `~/robot_ws/src/my_robot_description/config/`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_drive_controller:
  ros__parameters:
    left_wheel_names: [left_wheel_joint]
    right_wheel_names: [right_wheel_joint]
    wheel_separation: 0.24
    wheel_radius: 0.05
    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_link
    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    cmd_vel_timeout: 0.5
    # publish_limited_velocity: true
    # use_stamped_vel: false
    # enable_odom_tf: true

    # Velocity and acceleration limits
    linear:
      x:
        has_velocity_limits: true
        max_velocity: 0.5
        has_acceleration_limits: true
        max_acceleration: 1.0
    angular:
      z:
        has_velocity_limits: true
        max_velocity: 1.0
        has_acceleration_limits: true
        max_acceleration: 2.0
```

## 4. Create the Gazebo World File (`lawn.sdf`)

Navigate to `~/robot_ws/src/my_robot_bringup/worlds` and create `lawn.sdf`. This world will include a simple ground plane and your robot.

### `lawn.sdf`

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="lawn_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Your Robot -->
    <include>
      <uri>model://my_robot</uri>
      <name>my_robot</name>
      <pose>0 0 0.1 0 0 0</pose> <!-- Adjust initial pose as needed -->
    </include>

  </world>
</sdf>
```

**Note:** For `model://my_robot` to work, you need to tell Gazebo where to find your robot model. This is typically done by setting the `GAZEBO_MODEL_PATH` environment variable. You'll define this in your launch file.

## 5. Create Launch Files

### `display_robot.launch.py` (in `~/robot_ws/src/my_robot_description/launch/`)

This launch file will display your robot in RViz2.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get URDF file path
    urdf_file_name = 'my_robot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        urdf_file_name
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # Joint State Publisher (GUI for manual joint control)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # RViz2
    rviz_config_dir = os.path.join(get_package_share_directory('my_robot_description'), 'rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(rviz_config_dir, 'my_robot.rviz')],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
```

### `spawn_robot.launch.py` (in `~/robot_ws/src/my_robot_bringup/launch/`)

This launch file will spawn your robot in Gazebo and load the controllers.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get URDF file path
    urdf_file_name = 'my_robot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        urdf_file_name
    )

    # Set GAZEBO_MODEL_PATH to include your robot's directory
    gazebo_model_path = os.path.join(get_package_share_directory('my_robot_description'), 'models')
    os.environ['GAZEBO_MODEL_PATH'] = os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + gazebo_model_path

    # Gazebo launch
    gazebo_launch_file_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    gazebo_world_path = os.path.join(get_package_share_directory('my_robot_bringup'), 'worlds', 'lawn.sdf')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_file_dir, 'gazebo.launch.py')),
        launch_arguments={'world': gazebo_world_path}.items(),
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

    # Load controllers
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

    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + gazebo_model_path),
        gazebo_cmd,
        robot_state_publisher_node,
        spawn_entity,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
    ])
```

## 6. Build and Run

```bash
cd ~/robot_ws
colcon build
source install/setup.bash

# Launch Gazebo with your robot
ros2 launch my_robot_bringup spawn_robot.launch.py

# In a new terminal, you can launch RViz2 to visualize the robot (optional)
ros2 launch my_robot_description display_robot.launch.py
```

## 7. Control the Robot

Once Gazebo is running, you can control the robot using the `/my_robot/cmd_vel` topic:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}' -r 10
```

## 8. View Camera Feed

You can view the camera feed using RViz2 or `rqt_image_view`:

```bash
ros2 run rqt_image_view rqt_image_view /my_robot/camera/image_raw
```

This provides a basic setup. You might need to adjust parameters, add more sophisticated controllers, or refine the URDF for more complex behaviors. Remember to rebuild your workspace (`colcon build`) and re-source the setup files (`source install/setup.bash`) after making changes to your URDF or launch files.