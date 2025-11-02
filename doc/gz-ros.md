# Creating a ROS2-controlled Robot in Gazebo

This guide outlines the steps to create a robot model described in URDF/XACRO, include it in a Gazebo world, and enable ROS2 control with a camera sensor. This documentation reflects a working configuration with all common issues resolved.

## 1. Prerequisites

Ensure you have ROS2 (we use Humble) and Gazebo 11 installed and sourced correctly.

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
```

### Required Packages

Install the necessary ROS 2 control packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-diff-drive-controller \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-ros-pkgs
```

## 2. Create a ROS2 Workspace

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
ros2 pkg create --build-type ament_cmake my_robot_description
ros2 pkg create --build-type ament_cmake my_robot_bringup
cd ..
colcon build --symlink-install
source install/setup.bash
```

## 3. Define the Robot Model (URDF/XACRO)

Navigate to `~/robot_ws/src/my_robot_description/urdf` and create `my_robot.urdf.xacro`. Using XACRO allows parameterization and better organization.

### Key URDF/XACRO Features

#### Base Structure
- **base_footprint**: Ground reference frame
- **base_link**: Main robot body (0.6m x 0.4m x 0.25m)
- **left_wheel_link** and **right_wheel_link**: Drive wheels (radius: 0.1m, separation: 0.44m)
- **caster_wheel_link**: Rear support sphere (radius: 0.06m)
- **camera_link**: Front-mounted camera

#### Critical Gazebo Physics Properties

For wheels to have proper traction, add friction coefficients:

```xml
<gazebo reference="left_wheel_link">
  <material>Gazebo/Black</material>
  <mu1>1.0</mu1>  <!-- Friction coefficient in primary direction -->
  <mu2>1.0</mu2>  <!-- Friction coefficient in secondary direction -->
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>100.0</kd>  <!-- Contact damping -->
  <minDepth>0.001</minDepth>
  <maxVel>1.0</maxVel>
</gazebo>
```

**Without these friction parameters, wheels will spin but the robot won't move (like on ice)!**

#### ROS 2 Control Integration

Use `ros2_control` framework instead of the deprecated `libgazebo_ros_diff_drive.so`:

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-10</param>
      <param name="max">10</param>
    </command_interface>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-10</param>
      <param name="max">10</param>
    </command_interface>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
</ros2_control>

<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_robot_description)/config/my_robot_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

**Important**: Use `$(find package_name)` syntax, NOT `${variable}` for the parameters path!

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
    # Wheel parameters that MUST match the URDF
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.44  # Distance between wheel centers
    wheel_radius: 0.1       # Wheel radius in meters

    # Odometry
    odom_frame_id: "odom"
    base_frame_id: "base_footprint"
    publish_rate: 50.0

    # Velocity command timeout
    cmd_vel_timeout: 0.5

    # Velocity and acceleration limits
    linear:
      x:
        has_velocity_limits: true
        max_velocity: 1.0
        has_acceleration_limits: true
        max_acceleration: 1.0
    angular:
      z:
        has_velocity_limits: true
        max_velocity: 1.5
        has_acceleration_limits: true
        max_acceleration: 2.0
```

## 4. Create the Gazebo World File (`lawn.sdf`)

Navigate to `~/robot_ws/src/my_robot_bringup/worlds` and create `lawn.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="lawn_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

**Note**: The robot is spawned programmatically via the launch file, not included in the world file.

## 5. Create Launch Files

### `spawn_robot.launch.py` (in `~/robot_ws/src/my_robot_bringup/launch/`)

```python
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

    # Load joint state broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Load differential drive controller
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
```

### Twist Stamper Script

The `diff_drive_controller` expects `TwistStamped` messages with `BEST_EFFORT` QoS, but standard teleop tools publish `Twist` messages. Create a bridge node:

Create `~/robot_ws/src/my_robot_bringup/scripts/twist_stamper.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')

        # QoS profile matching the controller's expectation
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribe to unstamped cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publish stamped cmd_vel with BEST_EFFORT QoS
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            qos_profile
        )

        self.get_logger().info('Twist Stamper node started - bridging /cmd_vel to /diff_drive_controller/cmd_vel')

    def cmd_vel_callback(self, msg):
        # Convert Twist to TwistStamped
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg

        # Publish
        self.publisher.publish(stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make it executable:
```bash
chmod +x ~/robot_ws/src/my_robot_bringup/scripts/twist_stamper.py
```

Update `CMakeLists.txt` in `my_robot_bringup`:
```cmake
install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/twist_stamper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

## 6. Build and Run

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

# Launch Gazebo with your robot
ros2 launch my_robot_bringup spawn_robot.launch.py
```

## 7. Control the Robot

Once Gazebo is running, control the robot using `/cmd_vel`:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10

# Turn in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}' -r 10

# Move forward while turning
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}' -r 10
```

### Using Teleop Keyboard

For interactive control:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### View robot in rviz

View robot in rviz2 environment
```bash
rviz2 -d ~/robot_ws/install/my_robot_bringup/share/my_robot_bringup/rviz/robot_view.rviz
```

## 8. View Camera Feed

View the camera feed using RViz2 or `rqt_image_view`:

```bash
ros2 run rqt_image_view rqt_image_view /my_robot/camera/image_raw
```

## 9. Monitoring and Debugging

### Check Controller Status
```bash
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster[active]
diff_drive_controller  [active]
```

### Monitor Joint States
```bash
ros2 topic echo /joint_states
```

### Check Odometry
```bash
ros2 topic echo /diff_drive_controller/odom
```

### Verify Message Flow
```bash
# Check if twist_stamper is converting messages
ros2 topic hz /cmd_vel
ros2 topic hz /diff_drive_controller/cmd_vel

# Check message types
ros2 topic info /cmd_vel -v
ros2 topic info /diff_drive_controller/cmd_vel -v
```

## 10. Common Issues and Solutions

### Issue 1: gazebo_ros2_control Parameter Errors

**Error:**
```
[ERROR] [gazebo_ros2_control]: No parameter file provided
[ERROR] [gazebo_ros2_control]: failed to parse input yaml file(s)
```

**Solution:**
Use `$(find package_name)` syntax in URDF, not `${variable}`:
```xml
<parameters>$(find my_robot_description)/config/my_robot_controllers.yaml</parameters>
```

### Issue 2: Gazebo Crashes with "exit code 255"

**Error:**
```
[ERROR] [gzserver-1]: process has died [pid XXX, exit code 255]
```

**Solution:**
The `libgazebo_ros_force_system.so` plugin causes crashes. Disable it in the launch file:
```python
launch_arguments={
    'world': gazebo_world_path,
    'force_system': 'false',  # Critical fix!
}
```

### Issue 3: Robot Not Moving (Wheels Spinning in Place)

**Symptoms:** Wheels rotate but robot doesn't move, like on ice.

**Solution:**
Add friction coefficients to wheel links in URDF:
```xml
<gazebo reference="left_wheel_link">
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <minDepth>0.001</minDepth>
  <maxVel>1.0</maxVel>
</gazebo>
```

### Issue 4: QoS Incompatibility

**Symptoms:** Messages published but controller doesn't receive them.

**Problem:** The `diff_drive_controller` subscribes with `BEST_EFFORT` QoS, but publishers use `RELIABLE` by default.

**Solution:** Use the twist_stamper bridge node (provided above) with proper QoS settings.

### Issue 5: Controller Not Found

**Error:**
```
Loader for controller 'joint_state_broadcaster' not found
```

**Solution:**
Install missing ROS 2 control packages:
```bash
sudo apt install ros-humble-ros2-controllers ros-humble-joint-state-broadcaster
```

## 11. Project Structure

```
robot_ws/
├── src/
│   ├── my_robot_description/
│   │   ├── config/
│   │   │   └── my_robot_controllers.yaml
│   │   ├── urdf/
│   │   │   └── my_robot.urdf.xacro
│   │   ├── launch/
│   │   ├── rviz/
│   │   └── CMakeLists.txt
│   └── my_robot_bringup/
│       ├── launch/
│       │   └── spawn_robot.launch.py
│       ├── worlds/
│       │   └── lawn.sdf
│       ├── scripts/
│       │   └── twist_stamper.py
│       └── CMakeLists.txt
└── doc/
    └── gz-ros.md
```

## 12. Additional Resources

- [ROS 2 Control Documentation](https://control.ros.org/humble/index.html)
- [Gazebo ROS 2 Control](https://github.com/ros-simulation/gazebo_ros2_control)
- [diff_drive_controller Documentation](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [Gazebo Classic Migration Guide](https://gazebosim.org/docs/latest/gazebo_classic_migration/)

## 13. Next Steps

- Add compass sensor
- Implement SLAM and navigation using Nav2
- Add autonomous behaviors
- Create custom world environments
- Implement computer vision using the camera feed
