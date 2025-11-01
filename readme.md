# Example for connecting gazebo classic with ros2

# ROS 2 Differential Drive Robot

A ROS 2 Humble project featuring a differential drive mobile robot with Gazebo Classic simulation.

## Features
- URDF/XACRO robot description
- Gazebo Classic integration
- ros2_control with diff_drive_controller
- Camera sensor
- Teleop keyboard control

## Quick Start
```bash
# Build
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

# Launch
ros2 launch my_robot_bringup spawn_robot.launch.py

# Control (in new terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# Documentation
 See [doc/gz-ros.md](./doc/gz-ros.md) for complete setup guid and troubleshooting.

