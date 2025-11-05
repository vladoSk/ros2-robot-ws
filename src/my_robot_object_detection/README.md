# My Robot Object Detection

Object detection package for my_robot using OpenCV and ROS2. This package implements color-based object detection (default: red objects) and robot following behavior.

## Features

- **Color-based object detection** using HSV color space
- **Real-time video processing** with OpenCV
- **Object tracking** with bounding boxes and center point visualization
- **Autonomous following** behavior using proportional control
- **Configurable parameters** via YAML files
- **Multiple ROS2 topics** for detection data and annotated images

## Nodes

### 1. object_detector

Detects colored objects in camera feed and publishes detection results.

**Subscribed Topics:**
- `/my_robot/camera/image_raw` (sensor_msgs/Image) - Camera feed from robot

**Published Topics:**
- `/object_detection/image_annotated` (sensor_msgs/Image) - Annotated image with bounding boxes
- `/object_detection/center` (geometry_msgs/Point) - Object center coordinates (x, y) and area (z)
- `/object_detection/detected` (std_msgs/Bool) - Detection status

**Parameters:**
- `lower_hue_1`, `lower_saturation_1`, `lower_value_1` - Lower HSV bound (range 1)
- `upper_hue_1`, `upper_saturation_1`, `upper_value_1` - Upper HSV bound (range 1)
- `lower_hue_2`, `lower_saturation_2`, `lower_value_2` - Lower HSV bound (range 2)
- `upper_hue_2`, `upper_saturation_2`, `upper_value_2` - Upper HSV bound (range 2)
- `min_area` - Minimum contour area to consider as valid detection
- `camera_topic` - Camera topic to subscribe to
- `show_debug` - Enable debug logging

### 2. object_follower

Makes the robot follow detected objects using proportional control.

**Subscribed Topics:**
- `/object_detection/detected` (std_msgs/Bool) - Detection status
- `/object_detection/center` (geometry_msgs/Point) - Object center and area

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to robot

**Parameters:**
- `enabled` - Enable/disable follower
- `linear_speed` - Maximum linear velocity (m/s)
- `angular_speed` - Maximum angular velocity (rad/s)
- `image_width`, `image_height` - Camera resolution
- `center_tolerance` - Pixels from center to consider "centered"
- `min_area`, `max_area` - Area thresholds for distance control
- `kp_angular`, `kp_linear` - Proportional gains

## Installation

```bash
cd ~/ros2-robot-ws
colcon build --packages-select my_robot_object_detection
source install/setup.bash
```

## Usage

### 1. Detection Only (No Robot Movement)

Start the full system with detection only:

```bash
ros2 launch my_robot_object_detection full_system.launch.py
```

Or start just the detection node:

```bash
# Terminal 1: Start robot in Gazebo
ros2 launch my_robot_bringup spawn_robot.launch.py

# Terminal 2: Start object detection
ros2 launch my_robot_object_detection object_detection.launch.py enable_follower:=false
```

### 2. Detection with Following Behavior

**WARNING:** The robot will move automatically!

```bash
ros2 launch my_robot_object_detection full_system.launch.py enable_follower:=true
```

### 3. View Detection Results

```bash
# View annotated image with bounding boxes
ros2 run rqt_image_view rqt_image_view /object_detection/image_annotated

# View original camera feed
ros2 run rqt_image_view rqt_image_view /my_robot/camera/image_raw

# Monitor detection status
ros2 topic echo /object_detection/detected

# Monitor object center position
ros2 topic echo /object_detection/center
```

## Configuration

Edit `config/object_detection_params.yaml` to adjust detection parameters:

### Detecting Different Colors

**Red objects (default):**
```yaml
lower_hue_1: 0
upper_hue_1: 10
lower_hue_2: 170
upper_hue_2: 180
```

**Blue objects:**
```yaml
lower_hue_1: 100
upper_hue_1: 130
lower_hue_2: 100  # Same as range 1 (blue doesn't wrap)
upper_hue_2: 130
```

**Green objects:**
```yaml
lower_hue_1: 40
upper_hue_1: 80
lower_hue_2: 40
upper_hue_2: 80
```

**Yellow objects:**
```yaml
lower_hue_1: 20
upper_hue_1: 40
lower_hue_2: 20
upper_hue_2: 40
```

### Tuning Detection Sensitivity

- **Increase min_area** to detect only larger objects
- **Decrease min_area** to detect smaller objects
- **Adjust saturation/value ranges** for different lighting conditions

### Tuning Following Behavior

- **kp_angular**: Higher = more aggressive turning (default: 0.003)
- **kp_linear**: Higher = more aggressive forward/backward (default: 0.00001)
- **min_area/max_area**: Adjust desired following distance

## Testing in Gazebo

To test object detection, you need to add colored objects to the Gazebo world:

1. Open Gazebo (it should already be running)
2. Click "Insert" tab on the left panel
3. Add colored objects (boxes, spheres, cylinders)
4. Use the "Visual" properties to set red color (RGB: 1, 0, 0)
5. Place objects in front of the robot's camera

Alternatively, you can spawn objects programmatically or modify the world file.

## Troubleshooting

### No objects detected
- Check camera is working: `ros2 topic echo /my_robot/camera/image_raw`
- Verify object color matches HSV ranges
- Adjust min_area if objects appear too small
- Check lighting conditions in Gazebo

### Robot not following
- Ensure follower is enabled: `enable_follower:=true`
- Check velocity commands: `ros2 topic echo /cmd_vel`
- Verify object area is within min_area/max_area range
- Ensure twist_stamper node is running

### Poor detection accuracy
- Fine-tune HSV color ranges
- Adjust morphological operation kernel size
- Increase min_area to filter noise
- Improve Gazebo lighting

## Architecture

```
Gazebo Camera
    ↓
/my_robot/camera/image_raw
    ↓
object_detector node
    ↓
├─→ /object_detection/image_annotated (for visualization)
├─→ /object_detection/detected (Bool)
└─→ /object_detection/center (Point with area)
    ↓
object_follower node
    ↓
/cmd_vel
    ↓
twist_stamper
    ↓
/diff_drive_controller/cmd_vel
    ↓
Robot moves!
```

## Future Enhancements

- [ ] Deep learning-based object detection (YOLO, SSD)
- [ ] Multiple object tracking
- [ ] Object classification
- [ ] Obstacle avoidance while following
- [ ] Path planning integration
- [ ] Save/load detection parameters
- [ ] Dynamic parameter reconfiguration
- [ ] Performance metrics and logging

## License

Apache-2.0

## Author

ROS2 Robot Workspace
