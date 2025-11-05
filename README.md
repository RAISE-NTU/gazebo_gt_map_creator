# Gazebo GT Map Creator

A standalone ROS 2 package for generating 2D occupancy maps, 3D point clouds, and octomaps from Ignition Gazebo simulations.

## Features

- 🗺️ **2D Occupancy Maps**: Generate PGM and PNG maps compatible with Nav2
- 🎯 **3D Point Clouds**: Export PCD format point clouds
- 🧊 **Octomaps**: Create binary octomap (.bt) files
- 📐 **Geometry-based Ray Casting**: Accurate collision detection using axis-aligned bounding boxes
- ⚙️ **Configurable Parameters**: Adjust resolution, scanning area, and thresholds
- 🚀 **ROS 2 Service Interface**: Easy integration with ROS 2 systems
- 🔌 **Plugin Architecture**: Drop-in system plugin for Ignition Gazebo

## Output Formats

When you generate a map, the following files are created:

- `.pgm` - Grayscale occupancy map for ROS 2 Nav2 (**black obstacles, white free space**)
- `.png` - Color visualization of the map
- `.yaml` - Map metadata for Nav2 map_server (with relative path to .pgm)
- `.pcd` - 3D point cloud (ASCII format)
- `.bt` - Binary octomap for 3D navigation

**Nav2 Compatibility:** The plugin generates maps with proper Nav2 coloring:
- **Black (0)**: Occupied cells (walls, obstacles)
- **White (255)**: Free navigable space
- The YAML file uses relative paths for easy portability

## Dependencies

### Required
- ROS 2 (Humble or later)
- Ignition Gazebo (Fortress or later)
- `gazebo_map_creator_interface` package
- PCL (Point Cloud Library)
- OctoMap
- Boost (with GIL for image processing)
- libpng

### Installation

```bash
# Install system dependencies
sudo apt-get install \
    ros-${ROS_DISTRO}-ignition-gazebo6 \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    libboost-dev \
    libpcl-dev \
    liboctomap-dev \
    libpng-dev

# Clone the interface package (if not already in your workspace)
cd ~/your_ws/src
git clone <gazebo_map_creator_interface_repo>

# Clone this package
git clone <this_repo>

# Build
cd ~/your_ws
colcon build --packages-select gazebo_gt_map_creator
source install/setup.bash
```

## Usage

### 1. Add Plugin to Your World

Edit your world SDF file (e.g., `my_world.sdf`) and add the plugin:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="my_world">
    
    <!-- Add the map creator plugin -->
    <plugin
      filename="libgazebo_gt_map_creator_system.so"
      name="gazebo_gt_map_creator::GazeboGtMapCreator">
    </plugin>
    
    <!-- Your world content -->
    <model name="ground_plane">
      <!-- ... -->
    </model>
    
    <!-- Add your models, obstacles, etc. -->
    
  </world>
</sdf>
```

### 2. Launch Your Simulation

```bash
ign gazebo my_world.sdf
```

### 3. Generate the Map

#### Option A: Using Command-Line Arguments (Recommended)

```bash
# Basic usage
ros2 run gazebo_gt_map_creator save_map.py \
  --filename ~/maps/my_map \
  --upper-left -10 10 2 \
  --lower-right 10 -10 0 \
  --resolution 0.05

# Short options
ros2 run gazebo_gt_map_creator save_map.py \
  -f $PWD/map \
  -u -6 6 2 \
  -l 6 -6 0 \
  -r 0.05

# Fast 2D-only map (skip vertical scan)
ros2 run gazebo_gt_map_creator save_map.py \
  -f ~/maps/my_map \
  -u -10 10 2 \
  -l 10 -10 0 \
  -r 0.05 \
  -s

# View all options
ros2 run gazebo_gt_map_creator save_map.py --help
```

**Available Options:**
- `--filename, -f`: Output path (without extension)
- `--upper-left, -u`: X Y Z coordinates of upper left corner
- `--lower-right, -l`: X Y Z coordinates of lower right corner
- `--resolution, -r`: Meters per pixel (default: 0.05)
- `--threshold, -t`: Occupancy threshold 0-100 (default: 50)
- `--skip-vertical-scan, -s`: Flag for faster 2D-only maps
- `--range-multiplier, -m`: Ray casting range (default: 1.5)

#### Option B: Using the Python Script (Legacy)

```bash
ros2 service call /world/save_map gazebo_map_creator_interface/srv/MapRequest \
  "{upperleft: {x: -10.0, y: 10.0, z: 2.0}, 
    lowerright: {x: 10.0, y: -10.0, z: 0.0},
    resolution: 0.05,
    threshold_2d: 50,
    filename: '/tmp/my_map',
    skip_vertical_scan: true,
    range_multiplier: 1.5}"
```

#### Option C: Using Python Code

```python
import rclpy
from rclpy.node import Node
from gazebo_map_creator_interface.srv import MapRequest
from geometry_msgs.msg import Point

rclpy.init()
node = Node('my_map_creator')
client = node.create_client(MapRequest, '/world/save_map')
client.wait_for_service()

request = MapRequest.Request()
request.upperleft = Point(x=-10.0, y=10.0, z=2.0)
request.lowerright = Point(x=10.0, y=-10.0, z=0.0)
request.resolution = 0.05
request.filename = '/tmp/my_map'
request.skip_vertical_scan = True

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)

if future.result().success:
    print("Map created successfully!")
```

## Configuration Parameters

### Service Request Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `upperleft` | Point | - | Upper left corner (x, y, z) of map area |
| `lowerright` | Point | - | Lower right corner (x, y, z) of map area |
| `resolution` | float | 0.05 | Map resolution in meters per pixel |
| `threshold_2d` | int | 50 | Occupancy threshold (0-100) |
| `filename` | string | - | Output filename (without extension) |
| `skip_vertical_scan` | bool | false | Skip 3D scan for faster 2D maps |
| `range_multiplier` | float | 1.5 | Ray casting range multiplier |

### Understanding Coordinates

The coordinate system follows the standard robotics convention:

```
       Y (North)
       ↑
       |
       |
       +-----→ X (East)
      /
     /
    ↓
   Z (Down)
```

**Important:** The naming "upper-left" and "lower-right" refers to a top-down 2D view:
- **Upper Left** (`--upper-left`): **Minimum X**, **Maximum Y**, **Maximum Z** (ceiling)
- **Lower Right** (`--lower-right`): **Maximum X**, **Minimum Y**, **Minimum Z** (floor)

**Coordinate Validation Rules:**
- `upper_left.x < lower_right.x` (left is less than right)
- `upper_left.y > lower_right.y` (top is greater than bottom)  
- `upper_left.z > lower_right.z` (ceiling is higher than floor)

**Example for a 20x20 meter area centered at origin:**
```bash
# Correct coordinates
-u -10 10 2    # upper-left: x_min, y_max, z_max
-l 10 -10 0    # lower-right: x_max, y_min, z_min

# This creates a box from:
#   X: -10 to 10 (20 meters wide)
#   Y: -10 to 10 (20 meters long)
#   Z: 0 to 2 (2 meters tall)
```

## How It Works

1. **Collision Detection**: The plugin scans all static collision geometries in the world
2. **Geometry Support**: Handles boxes, cylinders, spheres, and meshes
3. **Ray Casting**: Uses axis-aligned bounding box (AABB) ray intersection for accurate detection
4. **Filtering**: Automatically filters out sensors, cameras, and lights
5. **Map Generation**: Creates a 2D occupancy grid by casting rays vertically
6. **3D Scanning**: Optionally scans in 6 directions (±X, ±Y, ±Z) for point cloud generation

## Tips for Best Results

### 1. Determine Your World Bounds

Before generating a map, identify the bounds of your world:

```bash
# List all models and their positions
ign model --list
ign model --model <model_name> --pose
```

### 2. Choose Appropriate Resolution

- **High detail** (0.01-0.02m): Small indoor environments
- **Standard** (0.05m): Most indoor environments
- **Low detail** (0.1-0.2m): Large outdoor environments

### 3. Skip Vertical Scan for 2D Maps

If you only need a 2D map for navigation, set `skip_vertical_scan: true` to speed up generation significantly.

### 4. Adjust Threshold

- Lower `threshold_2d` (e.g., 30): More conservative, marks more areas as occupied
- Higher `threshold_2d` (e.g., 70): More permissive, marks fewer areas as occupied

## Troubleshooting

### Map is empty or incorrect

1. Check your coordinates - ensure upper left is actually upper-left relative to lower-right
2. Verify the world has static collision geometries
3. Try increasing `range_multiplier` to 2.0 or higher: `-m 2.0`

### Map has grey obstacles instead of black

This has been fixed in the latest version. Rebuild the package:
```bash
colcon build --packages-select gazebo_gt_map_creator
source install/setup.bash
```
The plugin now generates Nav2-compatible maps with black (0) obstacles and white (255) free space.

### YAML file has wrong path to PGM

This has been fixed. The YAML now uses relative paths (just the filename) so maps are portable.

### Plugin not found

```bash
# Make sure the package is sourced
source ~/your_ws/install/setup.bash

# Verify plugin is installed
ls ~/your_ws/install/gazebo_gt_map_creator/lib/libgazebo_gt_map_creator_system.so
```

### Service not available

1. Ensure the simulation is running
2. Check the plugin is loaded: `ign gazebo -s` (look for GazeboGtMapCreator)
3. Verify ROS 2 bridge: `ros2 service list`

## Examples

### Small Room (5x5 meters)

```bash
ros2 run gazebo_gt_map_creator save_map.py \
  -f /tmp/small_room \
  -u -2.5 2.5 2.0 \
  -l 2.5 -2.5 0.0 \
  -r 0.02
```

### Large Warehouse (50x50 meters)

```bash
ros2 run gazebo_gt_map_creator save_map.py \
  -f /tmp/warehouse \
  -u -25 25 5 \
  -l 25 -25 0 \
  -r 0.1 \
  -s
```

### 3D Point Cloud Generation

```bash
ros2 run gazebo_gt_map_creator save_map.py \
  -f /tmp/3d_map \
  -u -10 10 3 \
  -l 10 -10 0 \
  -r 0.1
  # Don't use -s flag to enable 3D scanning
```

### Maze World Example

```bash
ros2 run gazebo_gt_map_creator save_map.py \
  -f $PWD/maze_map \
  -u -6 6 2 \
  -l 6 -6 0 \
  -r 0.05 \
  -s
```

## Using Generated Maps with Nav2

After generating a map:

```bash
# View the map
eog /tmp/my_map.png

# Launch Nav2 with your map
ros2 launch nav2_bringup navigation_launch.py \
  map:=/tmp/my_map.yaml
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please open an issue or pull request.

## Authors

- Kalhan Boralessa

## Acknowledgments

This project is based on [gazebo_map_creator](https://github.com/arshadlab/gazebo_map_creator) by arshadlab, licensed under Apache License 2.0. The original work provided the foundation for map creation from Gazebo simulations. This version has been significantly modified and extended for Ignition Gazebo compatibility, ROS 2 Humble integration, enhanced collision detection, Nav2 compatibility, and additional features.

## Support

For issues related to this Ignition Gazebo version, please open an issue in this repository.

---

This package provides a standalone version of map creation functionality for Ignition Gazebo, making it easy to integrate into any ROS 2 + Ignition Gazebo project.
