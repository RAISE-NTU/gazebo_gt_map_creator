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
  <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
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
# Check the PCD file header
head -n 20 /tmp/my_labeled_map.pcd

# Should show:
# FIELDS x y z label
# SIZE 4 4 4 4
# TYPE F F F U
```

## 5. Use the Labeled Point Cloud

### Python Example

```python
# Parse labeled PCD file
with open('/tmp/my_labeled_map.pcd', 'r') as f:
    lines = f.readlines()
    data_start = next(i for i, line in enumerate(lines) if line.startswith('DATA'))
    
    for line in lines[data_start + 1:]:
        x, y, z, label = map(float, line.strip().split())
        print(f"Point ({x:.2f}, {y:.2f}, {z:.2f}) -> Label {int(label)}")
```

## Label Numbering Schemes

### By Object Type
- 1 = Walls
- 2 = Furniture
- 3 = Doors
- 4 = Windows
- 5 = Obstacles

### By Room
- 100-199 = Kitchen
- 200-299 = Living Room
- 300-399 = Bedroom

### By Material
- 10 = Wood
- 20 = Metal
- 30 = Glass
- 40 = Plastic

### By Importance
- 0 = Background
- 10 = Low priority
- 50 = Medium priority
- 100 = High priority
- 255 = Critical

## Troubleshooting

**Q: All labels are 0?**
- Check that models have the label plugin
- Verify you used `-c` flag
- Make sure labels are on models, not links

**Q: How do I see what labels are in the world?**
```bash
# In Gazebo GUI, check the entity tree
# Or use CLI:
ign model --list
```

**Q: Can I change labels at runtime?**
- No, labels are set in the SDF file
- Restart Gazebo if you change labels

## Performance Tips

- Semantic labels add minimal overhead (~5%)
- File size increases by ~33% (4 bytes per point)
- No impact on 2D map generation

## Next Steps

- Read [SEMANTIC_LABELS.md](SEMANTIC_LABELS.md) for detailed examples
- See [example_world.sdf](worlds/example_world.sdf) for a working example
- Run [test_semantic_labels.sh](scripts/test_semantic_labels.sh) to test
