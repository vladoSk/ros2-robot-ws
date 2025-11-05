# Object Detection System for My Robot

This document describes the object detection system implemented for the my_robot project using OpenCV and ROS2.

## Overview

The object detection system enables the robot to:
- Detect colored objects in real-time using the onboard camera
- Track object positions and sizes
- Autonomously follow detected objects (optional)
- Publish annotated video feed with detection visualizations

## System Architecture

### Components

1. **object_detector node** - Core detection engine
   - Processes camera feed at 30 Hz
   - Uses HSV color space filtering
   - Applies morphological operations for noise reduction
   - Publishes detection results and annotated images

2. **object_follower node** - Autonomous behavior
   - Implements proportional control for smooth following
   - Maintains desired distance from object
   - Centers object in camera view
   - Publishes velocity commands

### Data Flow

```
┌─────────────────┐
│  Gazebo Camera  │
│   640x480 RGB   │
│     30 Hz       │
└────────┬────────┘
         │
         ↓ /my_robot/camera/image_raw
         │
┌────────┴─────────────────┐
│   object_detector node   │
│  - HSV color filtering   │
│  - Contour detection     │
│  - Bounding box calc     │
└────┬─────────────────────┘
     │
     ├─→ /object_detection/image_annotated ─→ [Visualization]
     │
     ├─→ /object_detection/detected ─┐
     │                                │
     └─→ /object_detection/center ────┤
                                      │
                              ┌───────┴────────────┐
                              │ object_follower    │
                              │ - P controller     │
                              │ - Distance control │
                              │ - Centering logic  │
                              └───────┬────────────┘
                                      │
                                      ↓ /cmd_vel
                                      │
                              ┌───────┴────────────┐
                              │  twist_stamper     │
                              └───────┬────────────┘
                                      │
                                      ↓ /diff_drive_controller/cmd_vel
                                      │
                              ┌───────┴────────────┐
                              │ DiffDriveController│
                              └───────┬────────────┘
                                      │
                                      ↓ Motor commands
                              ┌───────┴────────────┐
                              │  Robot wheels      │
                              └────────────────────┘
```

## Detection Algorithm

### Color-Based Detection (HSV)

The system uses HSV (Hue, Saturation, Value) color space for robust color detection:

1. **Color Space Conversion**
   - Input: BGR image from camera
   - Convert to HSV for better color separation
   - HSV is more invariant to lighting changes

2. **Color Filtering**
   - Apply two HSV range masks (red wraps around at 0/180)
   - Range 1: H=0-10, S=100-255, V=100-255
   - Range 2: H=170-180, S=100-255, V=100-255
   - Combine masks with bitwise OR

3. **Noise Reduction**
   - Erosion: Remove small noise (2 iterations)
   - Dilation: Restore object size (2 iterations)
   - Kernel: 5x5 square

4. **Contour Detection**
   - Find external contours in binary mask
   - Calculate contour areas
   - Filter by minimum area threshold

5. **Feature Extraction**
   - Bounding rectangle around largest contour
   - Center point (cx, cy) calculation
   - Area measurement for distance estimation

### Why HSV Instead of RGB?

| Aspect | RGB | HSV |
|--------|-----|-----|
| Color isolation | Difficult | Easy |
| Lighting invariance | Poor | Good |
| Intuitive ranges | No | Yes |
| Shadow handling | Poor | Better |

## Following Algorithm

### Proportional Control

The follower uses two independent P controllers:

#### Angular Control (Centering)
```
error_x = object_cx - image_center_x
angular_velocity = -kp_angular × error_x
```

- **Goal:** Keep object centered horizontally
- **Gain:** kp_angular = 0.003
- **Clamped:** ±0.5 rad/s

#### Linear Control (Distance)
```
if area < min_area:
    error = min_area - area
    linear_velocity = kp_linear × error  (forward)
elif area > max_area:
    error = max_area - area
    linear_velocity = kp_linear × error  (backward)
else:
    linear_velocity = 0  (maintain distance)
```

- **Goal:** Maintain object size (proxy for distance)
- **Gain:** kp_linear = 0.00001
- **Thresholds:** min_area=5000, max_area=100000 pixels

### Behavior States

```
┌──────────────────────────────────────┐
│         No Object Detected           │
│      State: STOPPED                  │
│      linear_x = 0, angular_z = 0     │
└──────────────┬───────────────────────┘
               │ Object appears
               ↓
┌──────────────────────────────────────┐
│      Object Detected (off-center)    │
│      State: CENTERING                │
│      linear_x = 0.3×forward          │
│      angular_z = P_angular(error_x)  │
└──────────────┬───────────────────────┘
               │ Object centered
               ↓
┌──────────────────────────────────────┐
│   Object Centered (distance wrong)   │
│   State: APPROACHING/RETREATING      │
│   linear_x = P_linear(area_error)    │
│   angular_z = P_angular(error_x)     │
└──────────────┬───────────────────────┘
               │ Distance optimal
               ↓
┌──────────────────────────────────────┐
│   Object Centered (distance good)    │
│   State: TRACKING                    │
│   linear_x = 0                       │
│   angular_z = P_angular(error_x)     │
└──────────────────────────────────────┘
```

## Topics Reference

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/my_robot/camera/image_raw` | sensor_msgs/Image | Raw camera feed (BGR8, 640x480, 30Hz) |
| `/object_detection/detected` | std_msgs/Bool | Detection status for follower |
| `/object_detection/center` | geometry_msgs/Point | Object center (x,y) and area (z) |

### Published Topics

| Topic | Type | Description | Rate |
|-------|------|-------------|------|
| `/object_detection/image_annotated` | sensor_msgs/Image | Annotated image with bounding boxes | 30 Hz |
| `/object_detection/detected` | std_msgs/Bool | Whether object is detected | 30 Hz |
| `/object_detection/center` | geometry_msgs/Point | Object center and area | 30 Hz |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands (follower) | 10 Hz |

## Parameters

### Object Detector

```yaml
# HSV Color Ranges (Red object default)
lower_hue_1: 0          # 0-180
upper_hue_1: 10
lower_saturation_1: 100  # 0-255
upper_saturation_1: 255
lower_value_1: 100       # 0-255
upper_value_1: 255

# Second range (for colors that wrap around)
lower_hue_2: 170
upper_hue_2: 180
lower_saturation_2: 100
upper_saturation_2: 255
lower_value_2: 100
upper_value_2: 255

# Detection
min_area: 500           # Minimum contour area (pixels²)
camera_topic: '/my_robot/camera/image_raw'
show_debug: true        # Enable debug logging
```

### Object Follower

```yaml
# Control
enabled: true
linear_speed: 0.2       # Max linear velocity (m/s)
angular_speed: 0.5      # Max angular velocity (rad/s)

# Image
image_width: 640
image_height: 480
center_tolerance: 50    # Pixels from center (considered centered)

# Distance control (via area)
min_area: 5000         # Too small = move forward
max_area: 100000       # Too large = move backward

# Gains
kp_angular: 0.003      # Angular velocity gain
kp_linear: 0.00001     # Linear velocity gain
```

## Usage Examples

### 1. Basic Object Detection (No Movement)

```bash
# Terminal 1: Launch robot in Gazebo
ros2 launch my_robot_bringup spawn_robot.launch.py

# Terminal 2: Start object detection only
ros2 launch my_robot_object_detection object_detection.launch.py enable_follower:=false

# Terminal 3: View annotated feed
ros2 run rqt_image_view rqt_image_view /object_detection/image_annotated
```

### 2. Full System with Following

```bash
# One command launches everything
ros2 launch my_robot_object_detection full_system.launch.py enable_follower:=true

# View results
ros2 run rqt_image_view rqt_image_view /object_detection/image_annotated
```

### 3. Manual Control with Detection Overlay

```bash
# Terminal 1: Launch full system without follower
ros2 launch my_robot_object_detection full_system.launch.py enable_follower:=false

# Terminal 2: Manual control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: View detection
ros2 run rqt_image_view rqt_image_view /object_detection/image_annotated
```

### 4. Monitoring Detection Data

```bash
# Watch detection status
ros2 topic echo /object_detection/detected

# Monitor object position
ros2 topic echo /object_detection/center

# Check detection rate
ros2 topic hz /object_detection/image_annotated
```

## Testing Object Detection

### Adding Test Objects in Gazebo

**Option 1: GUI Method**
1. Open Gazebo (should be running)
2. Click "Insert" tab in left panel
3. Select "Box" or "Sphere"
4. Click in world to place
5. Right-click object → Edit Model
6. Visual tab → Set Material color to Red (1, 0, 0)
7. File → Save As → Exit

**Option 2: Spawn via Command**
```bash
# Spawn red sphere at (2, 0, 0.5)
ros2 run gazebo_ros spawn_entity.py \
  -entity red_ball \
  -x 2 -y 0 -z 0.5 \
  -b \
  -database willowgarage_red_ball
```

**Option 3: Modify World File**
Edit `src/my_robot_bringup/worlds/lawn.sdf` to permanently add objects.

### Calibrating for Your Object

1. **Capture object image**
   ```bash
   ros2 topic echo /my_robot/camera/image_raw > /dev/null
   # Or use rqt_image_view to save image
   ```

2. **Find HSV values**
   - Use an HSV color picker tool
   - Or write a quick Python script:
   ```python
   import cv2
   import numpy as np

   # Load your object image
   img = cv2.imread('object.png')
   hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

   # Sample the object's color
   h, s, v = hsv[y, x]  # Click coordinates
   print(f"H: {h}, S: {s}, V: {v}")
   ```

3. **Update parameters**
   Edit `config/object_detection_params.yaml` with your ranges

4. **Test and iterate**
   ```bash
   # Rebuild if needed
   colcon build --packages-select my_robot_object_detection

   # Launch and observe
   ros2 launch my_robot_object_detection object_detection.launch.py
   ```

## Performance Considerations

### Computational Cost

- **Image conversion:** ~1 ms/frame
- **HSV conversion:** ~2 ms/frame
- **Color filtering:** ~3 ms/frame
- **Morphology:** ~4 ms/frame
- **Contour detection:** ~2-5 ms/frame (depends on complexity)
- **Drawing annotations:** ~1 ms/frame

**Total:** ~13-16 ms/frame → **60-75 Hz capable** (currently limited to 30 Hz by camera)

### Optimization Tips

1. **Reduce image resolution** if performance is an issue
   ```yaml
   # In camera plugin (URDF)
   <width>320</width>
   <height>240</height>
   ```

2. **Adjust morphology iterations**
   ```python
   # In object_detector.py
   mask = cv2.erode(mask, kernel, iterations=1)   # Reduce from 2
   mask = cv2.dilate(mask, kernel, iterations=1)
   ```

3. **Skip annotation publishing** when not viewing
   ```python
   # Add parameter
   publish_annotated: false
   ```

## Troubleshooting Guide

### Problem: No detections

**Symptoms:** "No Object Detected" always shown

**Solutions:**
1. Verify camera is working:
   ```bash
   ros2 topic hz /my_robot/camera/image_raw  # Should show ~30 Hz
   ```

2. Check object color matches HSV range:
   - Use `rqt_image_view` to inspect camera feed
   - Verify object is actually red (or your target color)
   - Lighting in Gazebo affects color perception

3. Lower `min_area` threshold:
   ```yaml
   min_area: 100  # Down from 500
   ```

4. Check if object is in field of view:
   - Camera FOV is 80° horizontal
   - Object must be within 0.02m - 300m range

### Problem: False positives

**Symptoms:** Detecting background or wrong objects

**Solutions:**
1. Narrow HSV ranges:
   ```yaml
   lower_saturation_1: 150  # Up from 100
   lower_value_1: 150       # Up from 100
   ```

2. Increase `min_area`:
   ```yaml
   min_area: 1000  # Up from 500
   ```

3. Improve Gazebo lighting (add directional light)

### Problem: Robot won't follow

**Symptoms:** Detection works but robot doesn't move

**Solutions:**
1. Verify follower is enabled:
   ```bash
   ros2 param get /object_follower enabled  # Should be True
   ```

2. Check cmd_vel is being published:
   ```bash
   ros2 topic echo /cmd_vel  # Should show velocity commands
   ```

3. Ensure twist_stamper is running:
   ```bash
   ros2 node list | grep twist_stamper
   ```

4. Verify object area is in range:
   ```bash
   ros2 topic echo /object_detection/center
   # Check z value (area) is between min_area and max_area
   ```

5. Check controller is loaded:
   ```bash
   ros2 control list_controllers
   # Should show diff_drive_controller [active]
   ```

### Problem: Erratic movement

**Symptoms:** Robot jerks, spins rapidly, or oscillates

**Solutions:**
1. Lower proportional gains:
   ```yaml
   kp_angular: 0.001  # Down from 0.003
   kp_linear: 0.000005  # Down from 0.00001
   ```

2. Increase center tolerance:
   ```yaml
   center_tolerance: 100  # Up from 50
   ```

3. Add velocity limits:
   ```yaml
   linear_speed: 0.1   # Down from 0.2
   angular_speed: 0.3  # Down from 0.5
   ```

4. Implement smoothing (future enhancement: low-pass filter)

### Problem: Jittery bounding boxes

**Symptoms:** Detection box jumps around

**Solutions:**
1. Increase morphology iterations:
   ```python
   iterations=3  # Up from 2
   ```

2. Add temporal filtering (moving average over last N frames)

3. Increase `min_area` to ignore noise

## Advanced Topics

### Adding More Sophisticated Detection

**Template Matching:**
```python
# Instead of color-based
template = cv2.imread('target_object.png', 0)
result = cv2.matchTemplate(gray_image, template, cv2.TM_CCOEFF_NORMED)
```

**Feature-Based (SIFT/ORB):**
```python
detector = cv2.ORB_create()
kp, des = detector.detectAndCompute(image, None)
```

**Deep Learning (YOLO via OpenCV):**
```python
net = cv2.dnn.readNet('yolov4.weights', 'yolov4.cfg')
blob = cv2.dnn.blobFromImage(image, 1/255, (416,416))
net.setInput(blob)
detections = net.forward()
```

### Multi-Object Tracking

To track multiple objects simultaneously:

1. Process all contours above threshold (not just largest)
2. Assign unique IDs to each detection
3. Implement tracking algorithm (Kalman filter, Hungarian algorithm)
4. Publish array of detections instead of single Point

### Integration with Nav2

To integrate with navigation stack:

1. Publish detected objects as obstacles in costmap
2. Use object location as navigation goal
3. Implement higher-level behavior state machine
4. Add path planning around detected objects

## Files Reference

```
my_robot_object_detection/
├── config/
│   └── object_detection_params.yaml     # All tunable parameters
├── launch/
│   ├── object_detection.launch.py       # Detection + follower
│   └── full_system.launch.py            # Robot + detection
├── my_robot_object_detection/
│   ├── __init__.py
│   ├── object_detector.py               # Detection node
│   └── object_follower.py               # Follower node
├── resource/
│   └── my_robot_object_detection
├── CMakeLists.txt
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## Related Documentation

- [Robot Setup Guide](gz-ros.md) - Setting up the robot in Gazebo
- [Future Development](lawn-mower-tiles.md) - Planned features
- [my_robot_object_detection README](../src/my_robot_object_detection/README.md) - Package documentation

## References

- OpenCV Documentation: https://docs.opencv.org/
- ROS2 Humble: https://docs.ros.org/en/humble/
- cv_bridge: http://wiki.ros.org/cv_bridge
- HSV Color Space: https://en.wikipedia.org/wiki/HSL_and_HSV

## Future Enhancements

- [ ] Implement Kalman filtering for smooth tracking
- [ ] Add multiple object tracking with unique IDs
- [ ] Integrate YOLO or other deep learning detector
- [ ] Add obstacle avoidance while following
- [ ] Implement PID control (add I and D terms)
- [ ] Dynamic reconfigure for parameters
- [ ] Recording and playback of detection sessions
- [ ] Performance metrics dashboard
- [ ] Web-based tuning interface
