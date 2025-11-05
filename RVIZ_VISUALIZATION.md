# Visualizing Point Clouds in RViz2

The `gazebo_gt_map_creator` now generates **colored RGB point clouds** that can be directly visualized in RViz2!

## Features

- 🎨 **Colored Point Clouds**: Points are colored based on semantic labels
- 🔍 **RViz2 Compatible**: Native RGB8 format support
- 🌈 **Automatic Color Assignment**: Each label gets a distinct color using HSV color space
- 📊 **Label Statistics**: Track unique colors/labels in the output

## Quick Start

### 1. Generate a Colored Point Cloud

```bash
# With semantic labels (colored by label)
ros2 run gazebo_gt_map_creator save_map.py \
  -f /tmp/colored_map \
  -u -10 10 2 \
  -l 10 -10 0 \
  -r 0.05 \
  -c

# Without labels (all points white)
ros2 run gazebo_gt_map_creator save_map.py \
  -f /tmp/white_map \
  -u -10 10 2 \
  -l 10 -10 0 \
  -r 0.05
```

### 2. Publish to RViz2

```bash
# Terminal 1: Publish the point cloud
ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py /tmp/colored_map.pcd

# Terminal 2: Launch RViz2
rviz2
```

### 3. Configure RViz2

1. **Set Fixed Frame**: Change to `map` in Global Options
2. **Add PointCloud2 Display**:
   - Click "Add" button
   - Select "PointCloud2"
   - Click "OK"
3. **Configure Display**:
   - Set Topic: `/point_cloud`
   - Set Color Transformer: **RGB8** (important!)
   - Adjust Size: 0.05m (or as needed)
   - Set Style: Flat Squares or Points

### 4. Use Pre-configured RViz2

```bash
# Launch with pre-configured settings
rviz2 -d $(ros2 pkg prefix gazebo_gt_map_creator)/share/gazebo_gt_map_creator/rviz/view_pointcloud.rviz

# Or find the config file
ros2 pkg prefix gazebo_gt_map_creator
# Then: <prefix>/share/gazebo_gt_map_creator/rviz/view_pointcloud.rviz
```

## Color Encoding

### With Semantic Labels (`--capture-labels`)

Each semantic label is automatically assigned a distinct color:

- **Label 0** (unlabeled): Gray (128, 128, 128)
- **Other labels**: HSV-based colors for maximum distinction

The algorithm uses the **golden angle** (137.508°) to distribute hues evenly:

```
Label 10  → Red-Orange
Label 20  → Yellow-Green
Label 30  → Cyan
Label 40  → Blue-Purple
Label 50  → Magenta
... and so on
```

### Without Semantic Labels

All points are colored **white (255, 255, 255)** for basic visualization.

## Advanced Usage

### Custom Publishing Rate

```bash
# Publish at 5 Hz
ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py /tmp/map.pcd --rate 5

# Publish at 0.5 Hz (every 2 seconds)
ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py /tmp/map.pcd --rate 0.5
```

### Custom Frame ID

```bash
# Use 'odom' frame instead of 'map'
ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py /tmp/map.pcd --frame odom
```

### All Options

```bash
ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py --help
```

## Point Cloud Format

The generated PCD files use the **PointXYZRGB** format:

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
2.0 0.0 0.5 16711680    # x y z rgb (rgb as uint32)
-2.0 2.0 0.5 65280
0.0 -2.0 0.5 255
```

The RGB values are packed into a single `uint32`:
```
rgb = (R << 16) | (G << 8) | B
```

## Viewing Options in RViz2

### Color Transformers

- **RGB8**: Use point cloud colors (default, recommended)
- **Intensity**: Grayscale based on intensity values
- **AxisColor**: Color by axis (X, Y, or Z)
- **FlatColor**: Single color for all points

### Style Options

- **Points**: Simple dots
- **Flat Squares**: Better visibility (recommended)
- **Spheres**: 3D spheres (more detailed)
- **Boxes**: Cubic representation

### Size Options

- **Size (Pixels)**: Fixed pixel size (good for dense clouds)
- **Size (m)**: Fixed meter size (scales with zoom)

## Example Workflow

### Complete Visualization Pipeline

```bash
# 1. Launch Gazebo with labeled world
ign gazebo worlds/example_world.sdf &

# 2. Wait for Gazebo to load, then generate map
sleep 5
ros2 run gazebo_gt_map_creator save_map.py \
  -f /tmp/semantic_map \
  -u -6 6 2 \
  -l 6 -6 0 \
  -r 0.05 \
  -s \
  -c

# 3. Publish to RViz2
ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py /tmp/semantic_map.pcd &

# 4. Launch RViz2 with config
rviz2 -d $(ros2 pkg prefix gazebo_gt_map_creator)/share/gazebo_gt_map_creator/rviz/view_pointcloud.rviz
```

## Troubleshooting

### Points appear all white in RViz2

**Solution**: Change Color Transformer to **RGB8**
1. Click on PointCloud2 display
2. Expand "Color Transformer" dropdown
3. Select "RGB8"

### Can't see any points

**Solutions**:
1. Check Fixed Frame matches publisher frame (default: `map`)
2. Increase point size: Set "Size (m)" to 0.1 or higher
3. Check topic: Should be `/point_cloud`
4. Verify publisher is running: `ros2 topic echo /point_cloud`

### Points are too small/large

**Solution**: Adjust size in RViz2
- For dense clouds: Use "Size (Pixels)" = 2-5
- For sparse clouds: Use "Size (m)" = 0.05-0.2

### Colors don't match labels

This is expected - colors are automatically generated using HSV distribution. To verify labels:

```bash
# Check the PCD file directly
head -n 30 /tmp/semantic_map.pcd
```

### RViz2 performance issues

**Solutions**:
1. Reduce publishing rate: `--rate 0.5`
2. Use "Flat Squares" instead of "Spheres"
3. Increase map resolution (larger resolution = fewer points)
4. Enable "Use Fixed Frame" in PointCloud2 display

## Integration with Nav2

You can visualize both the point cloud and navigation stack:

```bash
# Terminal 1: Navigation
ros2 launch nav2_bringup navigation_launch.py map:=/tmp/semantic_map.yaml

# Terminal 2: Point cloud publisher
ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py /tmp/semantic_map.pcd

# Terminal 3: RViz2 with Nav2 config
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

# Then add PointCloud2 display manually in RViz2
```

## Python API

You can also use the publisher programmatically:

```python
import rclpy
from your_package import PCDPublisher

rclpy.init()
node = PCDPublisher('/tmp/map.pcd', frame_id='map', rate=1.0)
rclpy.spin(node)
```

## Comparison: With vs Without Labels

| Feature | Without Labels | With Labels |
|---------|---------------|-------------|
| Color | All white | Colored by label |
| File Size | ~100 KB | ~100 KB (same) |
| RGB Field | Yes | Yes |
| Label Info | No | Encoded in color |
| RViz2 Viz | Mono | Multi-color |

## Next Steps

- **Save RViz2 Config**: File → Save Config As...
- **Record Bag Files**: `ros2 bag record /point_cloud`
- **Export Images**: RViz2 → Panels → Image → Save
- **Analyze Labels**: See [SEMANTIC_LABELS.md](SEMANTIC_LABELS.md)

## Related Tools

- **PCL Viewer**: `pcl_viewer /tmp/map.pcd`
- **CloudCompare**: Advanced point cloud visualization
- **Meshlab**: Mesh and point cloud processing
- **ROS Bag**: `ros2 bag record -a` to record all topics

## Tips

1. **High-quality screenshots**: Increase window size before capturing
2. **Better visibility**: Use dark background in RViz2 Global Options
3. **Performance**: Lower publishing rate for large point clouds
4. **Color schemes**: Modify `label_to_color()` in GazeboGtMapCreator.cpp for custom colors
