# Quick Start: Semantic Labels

## 1. Add Labels to Your World

Edit your world SDF file to add label plugins to models:

```xml
<model name="obstacle_1">
  <static>true</static>
  <pose>2 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </visual>
  </link>
  
  <!-- Add this -->
  <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
    <label>10</label>
  </plugin>
</model>
```

## 2. Launch Gazebo

```bash
ign gazebo your_world.sdf
```

## 3. Generate Labeled Map

```bash
ros2 run gazebo_gt_map_creator save_map.py \
  -f /tmp/my_labeled_map \
  -u -10 10 2 \
  -l 10 -10 0 \
  -r 0.05 \
  -c
```

## 4. Verify Output

```bash
# View colored point cloud in RViz2 (recommended)
ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py --file /tmp/my_labeled_map.pcd

# Or check the PCD file header
head -n 20 /tmp/my_labeled_map.pcd

# Should show:
# FIELDS x y z rgb
# SIZE 4 4 4 4
# TYPE F F F U
```

## 5. Use the Labeled Point Cloud

### Python Example with RViz2

```python
# Visualize in RViz2
ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py --file /tmp/my_labeled_map.pcd
```

### Python Example - Access RGB Colors

```python
import struct

# Parse RGB point cloud PCD file
with open('/tmp/my_labeled_map.pcd', 'r') as f:
    lines = f.readlines()
    data_start = next(i for i, line in enumerate(lines) if line.startswith('DATA'))
    
    for line in lines[data_start + 1:]:
        x, y, z, rgb_packed = line.strip().split()
        x, y, z = float(x), float(y), float(z)
        rgb_int = int(float(rgb_packed))
        
        # Unpack RGB components
        r = (rgb_int >> 16) & 0xFF
        g = (rgb_int >> 8) & 0xFF
        b = rgb_int & 0xFF
        
        print(f"Point ({x:.2f}, {y:.2f}, {z:.2f}) -> RGB({r}, {g}, {b})")
```
```

## Label Numbering Schemes

Different semantic labels are automatically assigned different colors for visualization in RViz2.

### By Object Type

- 1 = Walls (e.g., red)
- 2 = Furniture (e.g., green)
- 3 = Doors (e.g., blue)
- 4 = Windows (e.g., yellow)
- 5 = Obstacles (e.g., cyan)

### By Room

- 100-199 = Kitchen (e.g., orange shades)
- 200-299 = Living Room (e.g., purple shades)
- 300-399 = Bedroom (e.g., pink shades)

### By Material

- 10 = Wood (e.g., brown)
- 20 = Metal (e.g., silver)
- 30 = Glass (e.g., light blue)
- 40 = Plastic (e.g., white)

## Troubleshooting

**Q: All points are the same color (grey)?**

- Check that models have the label plugin
- Verify you used `-c` flag
- Make sure labels are on models, not links
- Verify plugin filename is `ignition-gazebo-label-system`

**Q: How do I see what labels are in the world?**

```bash
# In Gazebo GUI, check the entity tree
# Or use CLI:
ign model --list
```

**Q: Can I change labels at runtime?**

- No, labels are set in the SDF file
- Restart Gazebo if you change labels

**Q: Why use colors instead of storing label integers?**

- Colors enable direct visualization in RViz2
- No need for custom viewers
- Different labels get automatically distinct colors using HSV color space

## Performance Tips

- Semantic labels add minimal overhead (~5%)
- PointXYZRGB format is standard and widely supported
- File size same as regular RGB point clouds
- No impact on 2D map generation
- Colors automatically generated using HSV color space for visual distinction

## Next Steps

- Read [SEMANTIC_LABELS.md](SEMANTIC_LABELS.md) for detailed examples
- See [example_world.sdf](worlds/example_world.sdf) for a working example
- Run [test_semantic_labels.sh](scripts/test_semantic_labels.sh) to test
