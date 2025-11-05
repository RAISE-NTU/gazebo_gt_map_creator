# Quick Start Guide - Gazebo GT Map Creator

This guide will help you quickly get started with the gazebo_gt_map_creator plugin.

## Installation

```bash
# Navigate to your ROS 2 workspace
cd ~/your_ros2_workspace

# Build the package
colcon build --packages-select gazebo_gt_map_creator

# Source the workspace
source install/setup.bash
```

## Test the Package

### 1. Launch the Example World

```bash
# Make sure you've sourced the workspace
source install/setup.bash

# Launch the example world
ign gazebo src/gazebo_gt_map_creator/worlds/example_world.sdf
```

The simulation should start with a few obstacles (boxes, cylinder, and a wall).

### 2. Generate a Map

Open a new terminal and run:

```bash
# Source the workspace
source install/setup.bash

# For the example world, use these coordinates:
# The world spans approximately -5 to 5 in X and Y
ros2 run gazebo_gt_map_creator save_map.py \
  --filename /tmp/example_map \
  --upper-left -5 5 2 \
  --lower-right 5 -5 0 \
  --resolution 0.05
```

The map will be generated with **black obstacles** and **white free space**, compatible with Nav2.

### 3. View the Generated Map

```bash
# View the PNG visualization
eog /tmp/example_map.png

# Or view the PGM file
eog /tmp/example_map.pgm
```

## Use with Your Own World

### Step 1: Add the Plugin

Add this to your world SDF file:

```xml
<plugin
  filename="libgazebo_gt_map_creator_system.so"
  name="gazebo_gt_map_creator::GazeboGtMapCreator">
</plugin>
```

### Step 2: Determine World Bounds

Start your simulation and note the positions of objects to determine the map area.

### Step 3: Configure and Run

Call the service with your world's coordinates:

```bash
# Using the command-line interface (recommended)
ros2 run gazebo_gt_map_creator save_map.py \
  --filename /tmp/my_world_map \
  --upper-left -10 10 2 \
  --lower-right 10 -10 0 \
  --resolution 0.05

# Or use short options
ros2 run gazebo_gt_map_creator save_map.py \
  -f ~/maps/my_map \
  -u -10 10 2 \
  -l 10 -10 0 \
  -r 0.05 \
  -s  # Skip vertical scan for faster 2D maps

# Or call the service directly
ros2 service call /world/save_map gazebo_gt_map_creator/srv/MapRequest \
  "{upperleft: {x: -10.0, y: 10.0, z: 2.0}, 
    lowerright: {x: 10.0, y: -10.0, z: 0.0},
    resolution: 0.05,
    threshold_2d: 50,
    filename: '/tmp/my_world_map',
    skip_vertical_scan: true,
    range_multiplier: 1.5}"
```

## Troubleshooting

### "Service not available"
- Make sure your world is running
- Verify the plugin is in the SDF file
- Check that you've sourced the workspace

### "Empty map generated"
- Check your coordinate bounds
- Ensure objects in your world are **static**
- Verify objects have **collision** geometries

### "Plugin not found"
```bash
# Verify installation
ls install/gazebo_gt_map_creator/lib/libgazebo_gt_map_creator_system.so

# Re-source
source install/setup.bash
```

## Next Steps

- Read the full [README.md](README.md) for detailed documentation
- Adjust parameters for your specific use case
- Integrate with Nav2 for autonomous navigation

## Support

For issues or questions, please open an issue on the repository.
