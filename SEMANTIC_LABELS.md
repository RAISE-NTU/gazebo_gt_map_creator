# Semantic Label Capture

The `gazebo_gt_map_creator` plugin now supports capturing semantic labels in point clouds when models have the `ignition-gazebo-label-system` plugin attached.

## Overview

When enabled, the plugin will:
- Detect semantic labels from models using the Gazebo Label System plugin
- Generate RGB colored point clouds where each label gets a unique color
- Store point clouds in **PointXYZRGB** format (point with RGB color)
- Each semantic label automatically gets a distinct color using HSV color space
- Save colored point clouds in PCD format for visualization in RViz2

## Setup

### 1. Add Label Plugin to Your Models

Add the `ignition-gazebo-label-system` plugin to each model you want to label:

```xml
<model name="box1">
  <static>true</static>
  <pose>2 0 0.5 0 0 0</pose>
  <link name="link">
    <!-- ... collision and visual elements ... -->
  </link>
  
  <!-- Add semantic label plugin -->
  <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
    <label>10</label>
  </plugin>
</model>
```

Label values are `uint32` integers. You can use any numbering scheme that makes sense for your application:
- **1-10**: Different object types (box, cylinder, sphere, etc.)
- **10, 20, 30, ...**: Class-based labeling (obstacles, walls, furniture, etc.)
- **100-199**: Room 1 objects, **200-299**: Room 2 objects, etc.

### 2. Enable Semantic Label Capture

Use the `--capture-labels` or `-c` flag when generating maps:

```bash
# Basic usage with semantic labels
ros2 run gazebo_gt_map_creator save_map.py \
  -f ~/maps/labeled_map \
  -u -10 10 2 \
  -l 10 -10 0 \
  -r 0.05 \
  -c

# Combined with other options
ros2 run gazebo_gt_map_creator save_map.py \
  --filename $PWD/semantic_map \
  --upper-left -6 6 2 \
  --lower-right 6 -6 0 \
  --resolution 0.05 \
  --skip-vertical-scan \
  --capture-labels
```

## Output

When semantic labels are enabled, the generated PCD file will contain **PointXYZRGB** points with colors automatically assigned based on labels:

```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F U
COUNT 1 1 1 1
WIDTH 1000
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 1000
DATA ascii
2.0 0.0 0.5 16744192    # x y z rgb (label 10 = orange-red)
-2.0 2.0 0.5 6553600    # label 20 = yellow-green
0.0 -2.0 0.5 255        # label 30 = blue
...
```

**Color Assignment**: Each unique label gets a distinct color using HSV color space with the golden angle (137.508°) for maximum visual separation:
- Label 0 (unlabeled): Gray (128, 128, 128)
- Label 10: Red-Orange
- Label 20: Yellow-Green  
- Label 30: Cyan
- And so on...

## Using Labeled Point Clouds

### Visualize in RViz2

The easiest way to view labeled point clouds is with RViz2:

```bash
# Publish the colored point cloud
ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py labeled_map.pcd &

# Launch RViz2
rviz2 -d $(ros2 pkg prefix gazebo_gt_map_creator)/share/gazebo_gt_map_creator/rviz/view_pointcloud.rviz
```

**Important**: Set Color Transformer to **RGB8** in RViz2 to see the colors!

### Python with Open3D

```python
import open3d as o3d
import numpy as np

# Load the colored point cloud
pcd = o3d.io.read_point_cloud("labeled_map.pcd")
points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)

# Colors are already assigned based on labels
# Each unique label has a distinct color
print(f"Loaded {len(points)} points")
print(f"Color range: {colors.min()} to {colors.max()}")

# Visualize
o3d.visualization.draw_geometries([pcd])
```

### Python with PCL

```python
import pcl

# Load colored point cloud  
cloud = pcl.load("labeled_map.pcd")

# PointXYZRGB format: access x, y, z, and rgb
for i in range(min(cloud.size, 10)):  # First 10 points
    point = cloud[i]
    x, y, z, rgb = point[0], point[1], point[2], point[3]
    # Unpack RGB from uint32
    r = (int(rgb) >> 16) & 0xFF
    g = (int(rgb) >> 8) & 0xFF
    b = int(rgb) & 0xFF
    print(f"Point ({x:.2f}, {y:.2f}, {z:.2f}) RGB: ({r}, {g}, {b})")
```

### C++ with PCL

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("labeled_map.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    
    // Process colored points
    for (const auto& point : cloud->points) {
        std::cout << "Point (" << point.x << ", " << point.y << ", " 
                  << point.z << ") RGB: (" 
                  << (int)point.r << ", " << (int)point.g << ", " << (int)point.b 
                  << ")" << std::endl;
    }
    
    return 0;
}
```

## Applications

Semantic labeled point clouds are useful for:

1. **Visual Segmentation**: View different objects with distinct colors in RViz2
2. **Object Classification**: Identify object types based on color patterns
3. **Training Data Generation**: Create colored datasets for machine learning
4. **Multi-Robot Navigation**: Different robots can identify areas by color
5. **Semantic SLAM**: Build maps with color-coded object categories
6. **Collision Avoidance**: Different behaviors based on object colors/labels

## Example World

See `worlds/example_world.sdf` for a complete example with labeled models:

```xml
<model name="box1">
  <!-- ... -->
  <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
    <label>10</label>
  </plugin>
</model>

<model name="box2">
  <!-- ... -->
  <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
    <label>20</label>
  </plugin>
</model>
```

## Performance Considerations

- Semantic label capture has minimal performance impact
- The collision detection process is the same; color assignment is added
- PCD file size is the same (RGB already included in PointXYZRGB format)
- No impact on 2D map generation speed
- Color visualization works natively in RViz2 without extra processing

## Troubleshooting

### All points are gray

**Problem**: All points appear gray (128,128,128) in RViz2.

**Solutions**:
1. Ensure models have the `ignition-gazebo-label-system` plugin configured
2. Verify the `<label>` value is set in the plugin (not 0)
3. Check that you used the `--capture-labels` flag
4. Make sure Color Transformer is set to **RGB8** in RViz2

### Missing colors for some objects

**Problem**: Some objects don't show up with colors in the point cloud.

**Solutions**:
1. Make sure the label plugin is added to the model, not the link
2. Verify the model is marked as `<static>true</static>`
3. Check collision geometries are properly defined
4. Ensure plugin filename is `ignition-gazebo-label-system` (without .so)

### How to verify labels are working

Check the console output when generating the map:

```bash
# You should see messages like:
[INFO] Visual 'visual' has label: 100
[INFO] Saved labeled colored point cloud with 14478 points
[INFO] Point cloud contains 2 unique colors/labels
```

## Related Documentation

- [Gazebo Label System Plugin](https://gazebosim.org/api/sim/7/classgz_1_1sim_1_1systems_1_1Label.html)
- [PCL PointXYZRGB](https://pointclouds.org/documentation/classpcl_1_1_point_x_y_z_r_g_b.html)
- [RViz2 Visualization](RVIZ_VISUALIZATION.md)
- [Open3D Point Cloud](http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html)
