# Tile Scenarios

This directory contains text-based scenario files that define the layout of interactive tiles and walls for your robot simulation.

## File Format

Scenarios use a simple grid-based text format:

```
// Comments start with // (double slash) or ; (semicolon)
// Legend:
//   . = empty space (grass/floor)
//   T = interactive tile (disappears when robot steps on it)
//   # = brick wall (static obstacle)
//   R = robot spawn marker (optional, for reference only)

##########
#........#
#..TTTT..#
#..TTTT..#
#........#
##########
```

## Usage

### Option 1: With Launch File

Modify your launch file to include the scenario parameter:

```python
tile_manager = Node(
    package='my_robot_bringup',
    executable='tile_manager',
    name='tile_manager',
    output='screen',
    parameters=[{
        'tile_spacing': 0.5,  # meters between each grid cell
        'mode': 'disappear',
        'detection_radius': 0.3,
        'scenario_file': '/path/to/scenario.txt'
    }]
)
```

### Option 2: Command Line

```bash
source install/setup.bash
ros2 run my_robot_bringup tile_manager --ros-args \
  -p scenario_file:=/path/to/your/scenario.txt \
  -p tile_spacing:=0.5
```

### Option 3: Using Package Path

```bash
ros2 run my_robot_bringup tile_manager --ros-args \
  -p scenario_file:=$(ros2 pkg prefix my_robot_bringup)/share/my_robot_bringup/scenarios/lawn_with_walls.txt
```

## Available Scenarios

- **lawn_with_walls.txt** - Large lawn area with brick wall boundaries (6x8 tiles)
- **simple_grid.txt** - Simple 4x4 grid of tiles with wall perimeter

## Creating Your Own Scenarios

1. Create a new `.txt` file in this directory
2. Draw your layout using the characters: `.`, `T`, `#`, `R`
3. Each character represents a 0.5m x 0.5m cell (adjustable via `tile_spacing` parameter)
4. Make sure all rows have the same length for proper alignment
5. Use comments (#) to document your scenario

## Grid Spacing

The `tile_spacing` parameter controls the distance between grid cells:
- Default: 0.5 meters
- Smaller values = denser grid
- Larger values = more spread out

## Tips

- Keep scenarios centered by having equal borders on all sides
- Use walls (`#`) to create boundaries and prevent robot from leaving the area
- Interactive tiles (`T`) only disappear in `mode: disappear`
- Empty cells (`.`) allow the robot to pass through without interaction
