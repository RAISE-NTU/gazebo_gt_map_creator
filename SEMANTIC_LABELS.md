# Semantic Label Capture

The `gazebo_gt_map_creator` plugin now supports capturing semantic labels in point clouds when models have the `gz-sim-label-system` plugin attached.

## Overview

When enabled, the plugin will:
- Detect semantic labels from models using the Gazebo Label System plugin
- Store labels in the point cloud as `PointXYZL` format (point with label)
- Each collision detected will record which semantic label it belongs to
- Save labeled point clouds in PCD format for downstream processing

## Setup

### 1. Add Label Plugin to Your Models

Add the `gz-sim-label-system` plugin to each model you want to label:

```xml
<model name="box1">
  <static>true</static>
  <pose>2 0 0.5 0 0 0</pose>
  <link name="link">
    <!-- ... collision and visual elements ... -->
  </link>
  
  <!-- Add semantic label plugin -->
  <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
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

When semantic labels are enabled, the generated PCD file will contain `PointXYZL` points:

```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z label
SIZE 4 4 4 4
TYPE F F F U
COUNT 1 1 1 1
WIDTH 1000
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 1000
DATA ascii
2.0 0.0 0.5 10
-2.0 2.0 0.5 20
0.0 -2.0 0.5 30
...
```

## Using Labeled Point Clouds

### Python with Open3D

```python
import open3d as o3d
import numpy as np

# Load the labeled point cloud
pcd = o3d.io.read_point_cloud("labeled_map.pcd")
points = np.asarray(pcd.points)

# Labels are stored separately in the PCD file
# You'll need to parse the PCD file manually to extract labels
with open("labeled_map.pcd", 'r') as f:
    lines = f.readlines()
    # Find the start of data
    data_start = next(i for i, line in enumerate(lines) if line.startswith('DATA'))
    data_lines = lines[data_start + 1:]
    
    labels = []
    for line in data_lines:
        parts = line.strip().split()
        if len(parts) == 4:
            labels.append(int(parts[3]))  # label is 4th column

labels = np.array(labels)

# Filter points by label
label_10_points = points[labels == 10]
label_20_points = points[labels == 20]

print(f"Points with label 10: {len(label_10_points)}")
print(f"Points with label 20: {len(label_20_points)}")
```

### Python with PCL

```python
import pcl

# Load labeled point cloud
cloud = pcl.load("labeled_map.pcd")

# Access points and labels
for i in range(cloud.size):
    point = cloud[i]
    x, y, z = point[0], point[1], point[2]
    label = point[3]
    print(f"Point ({x}, {y}, {z}) has label {label}")
```

### C++ with PCL

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main() {
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZL>("labeled_map.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    
    // Process labeled points
    for (const auto& point : cloud->points) {
        std::cout << "Point (" << point.x << ", " << point.y << ", " 
                  << point.z << ") has label " << point.label << std::endl;
    }
    
    return 0;
}
```

## Applications

Semantic labeled point clouds are useful for:

1. **Object Segmentation**: Separate different objects in the scene
2. **Scene Understanding**: Identify object types and relationships
3. **Training Data Generation**: Create labeled datasets for machine learning
4. **Multi-Robot Navigation**: Different robots can focus on different labeled areas
5. **Semantic SLAM**: Build maps with object-level understanding
6. **Collision Avoidance**: Treat different objects with different strategies based on labels

## Example World

See `worlds/example_world.sdf` for a complete example with labeled models:

```xml
<model name="box1">
  <!-- ... -->
  <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
    <label>10</label>
  </plugin>
</model>

<model name="box2">
  <!-- ... -->
  <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
    <label>20</label>
  </plugin>
</model>
```

## Performance Considerations

- Semantic label capture has minimal performance impact
- The collision detection process is the same; only label storage is added
- PCD file size increases slightly (4 bytes per point for the label)
- No impact on 2D map generation speed

## Troubleshooting

### Labels are all zeros

**Problem**: All points have label 0 in the output PCD file.

**Solutions**:
1. Ensure models have the `gz-sim-label-system` plugin configured
2. Verify the `<label>` value is set in the plugin
3. Check that you used the `--capture-labels` flag

### Missing labels for some objects

**Problem**: Some objects don't have labels in the point cloud.

**Solutions**:
1. Make sure the label plugin is added to the model, not the link
2. Verify the model is marked as `<static>true</static>`
3. Check collision geometries are properly defined

### How to verify labels are working

Before running map generation, you can verify labels are properly configured:

```bash
# Launch Gazebo
ign gazebo worlds/example_world.sdf

# In another terminal, list all entities and their components
ign model --list
```

The models should show the Label component when properly configured.

## Future Enhancements

Planned improvements for semantic label support:

- [ ] Color-coded visualization of labels in PNG output
- [ ] Label-specific occupancy maps (separate map per label)
- [ ] Statistical output showing point distribution by label
- [ ] Support for hierarchical labels
- [ ] Export to semantic segmentation formats (e.g., COCO, Pascal VOC)

## Related Documentation

- [Gazebo Label System Plugin](https://gazebosim.org/api/sim/7/classgz_1_1sim_1_1systems_1_1Label.html)
- [PCL Point Types](https://pointclouds.org/documentation/point__types_8hpp_source.html)
- [Open3D Point Cloud](http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html)
