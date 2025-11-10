# Interactive Floor Tiles in Gazebo

This guide explains how to create floor tiles that disappear or change color when your robot steps on them.

## Overview

We'll implement a system where:
- Tiles are placed on the ground in a grid pattern
- When the robot's position overlaps a tile, the tile reacts
- Tiles can either disappear or change color


## Implementation: ROS2 Tile Manager Node

### Step 1: Create Tile Models

**File:** `src/my_robot_description/models/tile/model.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="floor_tile">
    <static>true</static>
    <link name="tile_link">
      <pose>0 0 0.01 0 0 0</pose>

      <!-- Visual appearance -->
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.02</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.8 0.2 1</ambient>  <!-- Green -->
          <diffuse>0.2 0.8 0.2 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>

      <!-- Optional collision (for physics-based detection) -->
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.02</size>
          </box>
        </geometry>
      </collision>

      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

**File:** `src/my_robot_description/models/tile/model.config`

```xml
<?xml version="1.0"?>
<model>
  <name>Floor Tile</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your@email.com</email>
  </author>
  <description>
    Interactive floor tile that can change color or disappear
  </description>
</model>
```

### Step 2: Create Tile Manager Node

**File:** `src/my_robot_bringup/scripts/tile_manager.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import math
import os
from ament_index_python.packages import get_package_share_directory


class TileManager(Node):
    def __init__(self):
        super().__init__('tile_manager')

        # Parameters
        self.declare_parameter('grid_size', 5)  # 5x5 grid
        self.declare_parameter('tile_spacing', 1.0)  # 1 meter between tiles
        self.declare_parameter('mode', 'disappear')  # 'disappear' or 'color_change'
        self.declare_parameter('detection_radius', 0.3)  # Distance to trigger tile

        self.grid_size = self.get_parameter('grid_size').value
        self.tile_spacing = self.get_parameter('tile_spacing').value
        self.mode = self.get_parameter('mode').value
        self.detection_radius = self.get_parameter('detection_radius').value

        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Subscribe to robot odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        # Track tiles
        self.tiles = {}  # {tile_name: {'x': x, 'y': y, 'active': True}}
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Wait for services
        self.get_logger().info('Waiting for Gazebo services...')
        self.spawn_client.wait_for_service(timeout_sec=10.0)
        self.delete_client.wait_for_service(timeout_sec=10.0)
        self.set_state_client.wait_for_service(timeout_sec=10.0)

        # Spawn tiles after a short delay
        self.create_timer(2.0, self.spawn_tiles_once)

        self.get_logger().info(f'Tile Manager initialized: {self.grid_size}x{self.grid_size} grid, mode={self.mode}')

    def spawn_tiles_once(self):
        """Spawn tiles once at startup"""
        self.destroy_timer(self.spawn_timer)  # Only run once
        self.spawn_tiles()

    def spawn_tiles(self):
        """Spawn a grid of tiles"""
        # Load tile model SDF
        model_path = os.path.join(
            get_package_share_directory('my_robot_description'),
            'models', 'tile', 'model.sdf'
        )

        try:
            with open(model_path, 'r') as f:
                tile_sdf = f.read()
        except FileNotFoundError:
            self.get_logger().error(f'Tile model not found at {model_path}')
            return

        # Calculate grid center offset
        offset = (self.grid_size - 1) * self.tile_spacing / 2.0

        for i in range(self.grid_size):
            for j in range(self.grid_size):
                x = i * self.tile_spacing - offset
                y = j * self.tile_spacing - offset

                tile_name = f'tile_{i}_{j}'

                # Create spawn request
                req = SpawnEntity.Request()
                req.name = tile_name
                req.xml = tile_sdf
                req.initial_pose = Pose()
                req.initial_pose.position.x = x
                req.initial_pose.position.y = y
                req.initial_pose.position.z = 0.0

                # Spawn tile
                future = self.spawn_client.call_async(req)
                future.add_done_callback(lambda f, tn=tile_name: self.spawn_callback(f, tn))

                # Track tile
                self.tiles[tile_name] = {
                    'x': x,
                    'y': y,
                    'active': True,
                    'triggered': False
                }

        self.get_logger().info(f'Spawned {self.grid_size * self.grid_size} tiles')

    def spawn_callback(self, future, tile_name):
        """Callback for spawn service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().debug(f'Spawned {tile_name}')
            else:
                self.get_logger().warn(f'Failed to spawn {tile_name}: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def odom_callback(self, msg):
        """Track robot position"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Check which tiles are under the robot
        self.check_tile_triggers()

    def check_tile_triggers(self):
        """Check if robot is on any tiles"""
        for tile_name, tile_data in self.tiles.items():
            if not tile_data['active']:
                continue

            # Calculate distance from robot to tile center
            dx = self.robot_x - tile_data['x']
            dy = self.robot_y - tile_data['y']
            distance = math.sqrt(dx*dx + dy*dy)

            # Check if robot is on tile
            if distance < self.detection_radius and not tile_data['triggered']:
                self.trigger_tile(tile_name, tile_data)
                tile_data['triggered'] = True

    def trigger_tile(self, tile_name, tile_data):
        """React when robot steps on tile"""
        self.get_logger().info(f'Robot stepped on {tile_name}!')

        if self.mode == 'disappear':
            self.delete_tile(tile_name)
            tile_data['active'] = False

        elif self.mode == 'color_change':
            self.change_tile_color(tile_name, tile_data)

    def delete_tile(self, tile_name):
        """Delete a tile from Gazebo"""
        req = DeleteEntity.Request()
        req.name = tile_name

        future = self.delete_client.call_async(req)
        future.add_done_callback(lambda f: self.delete_callback(f, tile_name))

    def delete_callback(self, future, tile_name):
        """Callback for delete service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Deleted {tile_name}')
            else:
                self.get_logger().warn(f'Failed to delete {tile_name}')
        except Exception as e:
            self.get_logger().error(f'Delete service call failed: {e}')

    def change_tile_color(self, tile_name, tile_data):
        """Change tile color (visual update)"""
        # Note: Gazebo doesn't support runtime material changes via services
        # Alternative: delete and respawn with different color, or use Gazebo topics
        self.get_logger().info(f'Tile {tile_name} color changed (placeholder)')
        # For now, just mark as triggered
        # Future implementation could delete and respawn with red color


def main(args=None):
    rclpy.init(args=args)
    node = TileManager()

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

### Step 3: Make Script Executable

```bash
chmod +x src/my_robot_bringup/scripts/tile_manager.py
```

### Step 4: Update CMakeLists.txt

**File:** `src/my_robot_bringup/CMakeLists.txt`

Add tile_manager.py to the install section:

```cmake
install(
  PROGRAMS
    scripts/twist_stamper.py
    scripts/tile_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# Also install the models directory
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)
```

### Step 5: Update package.xml

**File:** `src/my_robot_bringup/package.xml`

Add dependencies:

```xml
<depend>gazebo_msgs</depend>
<depend>nav_msgs</depend>
```

### Step 6: Create Launch File Integration

**Option A: Separate Launch File**

**File:** `src/my_robot_bringup/launch/spawn_robot_with_tiles.launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Include the base robot launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_bringup'),
                'launch',
                'spawn_robot.launch.py'
            )
        )
    )

    # Tile manager node
    tile_manager = Node(
        package='my_robot_bringup',
        executable='tile_manager.py',
        name='tile_manager',
        output='screen',
        parameters=[{
            'grid_size': 5,
            'tile_spacing': 1.0,
            'mode': 'disappear',  # or 'color_change'
            'detection_radius': 0.3
        }]
    )

    return LaunchDescription([
        robot_launch,
        tile_manager,
    ])
```

**Option B: Add to Existing Launch File**

Add to `spawn_robot.launch.py`:

```python
# Add at the end, before return LaunchDescription
tile_manager = Node(
    package='my_robot_bringup',
    executable='tile_manager.py',
    name='tile_manager',
    output='screen',
    parameters=[{
        'grid_size': 5,
        'tile_spacing': 1.0,
        'mode': 'disappear',
        'detection_radius': 0.3
    }]
)

# Add tile_manager to the launch description list
return LaunchDescription([
    gazebo_cmd,
    robot_state_publisher_node,
    spawn_entity,
    load_joint_state_broadcaster,
    load_diff_drive_controller,
    twist_stamper,
    tile_manager,  # Add this
])
```

### Step 7: Setup Model Path

Update your `.bashrc` or run before launching:

```bash
export GAZEBO_MODEL_PATH="$HOME/robot_ws/src/my_robot_description/models:$GAZEBO_MODEL_PATH"
```

Or create the models directory structure:

```bash
mkdir -p src/my_robot_description/models/tile
```

---

## Build and Test

### Build:

```bash
cd ~/robot_ws
colcon build --packages-select my_robot_bringup my_robot_description
source install/setup.bash
```

### Test:

```bash
# Launch robot with tiles
ros2 launch my_robot_bringup spawn_robot_with_tiles.launch.py

# Or with custom parameters
ros2 launch my_robot_bringup spawn_robot_with_tiles.launch.py grid_size:=7 mode:=disappear

# Test tile manager separately
ros2 run my_robot_bringup tile_manager.py --ros-args -p grid_size:=3
```

---

## Customization Options

### Change Grid Size

```python
parameters=[{'grid_size': 10}]  # 10x10 grid
```

### Change Tile Spacing

```python
parameters=[{'tile_spacing': 0.5}]  # Tiles 0.5m apart
```

### Change Mode

```python
parameters=[{'mode': 'color_change'}]  # Instead of disappearing
```

### Change Tile Appearance

Edit `models/tile/model.sdf` to modify:
- Color: Change `<ambient>` and `<diffuse>` RGB values
- Size: Change `<size>` in geometry
- Height: Adjust Z in `<pose>`

### Detection Sensitivity

```python
parameters=[{'detection_radius': 0.5}]  # Trigger from 0.5m away
```

---

## Advanced Features (Future Enhancements)

### 1. Color Changing Implementation

To actually change colors, you need to:
- Delete the old tile
- Spawn a new tile with different SDF (different color)

### 2. Score Tracking

Add to `TileManager`:

```python
self.score = 0
self.score_pub = self.create_publisher(Int32, '/tile_score', 10)

def trigger_tile(self, tile_name, tile_data):
    self.score += 1
    self.score_pub.publish(Int32(data=self.score))
    self.get_logger().info(f'Score: {self.score}')
```

### 3. Reset Functionality

Add service to respawn all tiles:

```python
self.reset_srv = self.create_service(Empty, '/reset_tiles', self.reset_callback)

def reset_callback(self, request, response):
    self.spawn_tiles()
    return response
```

### 4. Timed Respawn

Make tiles reappear after some time:

```python
def trigger_tile(self, tile_name, tile_data):
    self.delete_tile(tile_name)
    tile_data['active'] = False
    # Respawn after 5 seconds
    self.create_timer(5.0, lambda: self.respawn_tile(tile_name, tile_data))
```

### 5. Collision-Based Detection

Instead of position-based, use Gazebo contact sensors for precise detection.

---

## Troubleshooting

### Tiles Not Spawning

1. Check model path: `echo $GAZEBO_MODEL_PATH`
2. Verify model files exist: `ls src/my_robot_description/models/tile/`
3. Check Gazebo logs for errors

### Tiles Not Disappearing

1. Check robot odometry: `ros2 topic echo /diff_drive_controller/odom`
2. Verify tile manager is running: `ros2 node list | grep tile_manager`
3. Check detection radius parameter

### Performance Issues

- Reduce grid size
- Increase tile_spacing
- Use static models (they're more efficient)

---

## Alternative: Manual Tile Spawning

You can also spawn tiles manually via command:

```bash
# Get tile SDF content
TILE_SDF=$(cat src/my_robot_description/models/tile/model.sdf)

# Spawn a single tile at position (1, 1, 0)
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity \
  "{name: 'test_tile', xml: '$TILE_SDF', initial_pose: {position: {x: 1.0, y: 1.0, z: 0.0}}}"

# Delete a tile
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'test_tile'}"
```

---

## Complete File Checklist

- [ ] `src/my_robot_description/models/tile/model.sdf`
- [ ] `src/my_robot_description/models/tile/model.config`
- [ ] `src/my_robot_bringup/scripts/tile_manager.py` (executable)
- [ ] `src/my_robot_bringup/launch/spawn_robot_with_tiles.launch.py`
- [ ] Updated `src/my_robot_bringup/CMakeLists.txt`
- [ ] Updated `src/my_robot_bringup/package.xml`
- [ ] Set `GAZEBO_MODEL_PATH` environment variable

---

## Summary

This system provides:
- ✅ Grid-based tile spawning at startup
- ✅ Position-based detection when robot overlaps tiles
- ✅ Disappearing tiles mode
- ✅ Configurable parameters (grid size, spacing, mode)
- ✅ Easy to extend and customize

The tiles will automatically spawn when you launch your robot and disappear as your robot drives over them!
