# Changelog

All notable changes to the gazebo_gt_map_creator project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **Semantic Label Support**: Capture semantic labels from Gazebo's Label System plugin as RGB-colored point clouds
  - New `capture_semantic_labels` parameter in MapRequest service
  - Support for `PointXYZRGB` point cloud format (RGB-colored points for semantic visualization)
  - Automatic label detection from Visual entities with `ignition-gazebo-label-system` plugin
  - Automatic color assignment using HSV color space (golden angle distribution)
  - New `--capture-labels` / `-c` flag in save_map.py script
  - RViz2 visualization script (publish_pcd_to_rviz.py) for colored point cloud viewing
  - Pre-configured RViz2 config file (rviz/view_pointcloud.rviz)
  - Maze world example with color-coded semantic labels (worlds/maze.sdf)
  - Comprehensive documentation in SEMANTIC_LABELS.md, QUICKSTART_LABELS.md, and RVIZ_VISUALIZATION.md
  - Example world with labeled models (worlds/example_world.sdf)
  - Visualization screenshot in resource/semantic_labels.png

### Changed

- Updated collision box storage to include semantic label information
- Modified point cloud generation to use PointXYZRGB format with RGB colors encoding semantic labels
- Enhanced debug output to show detected semantic labels from Visual entities
- Updated README.md with semantic label feature overview, maze example, and visualization screenshot

### Technical Details

- Added `ignition::gazebo::components::SemanticLabel` component query for Visual entities
- Collision boxes now stored as tuples: `(bbox, pose, label)`
- New `CheckRayCollisionWithLabel()` method for label-aware collision detection
- Conditional point cloud type selection based on `capture_semantic_labels` flag
- Labels retrieved from Visual entities (Model → Link → Visual hierarchy)
- HSV to RGB color conversion using golden angle (137.508°) for distinct colors
- RGB values packed as uint32 for PCL PointXYZRGB compatibility

## [1.0.0] - 2025-11-05

### Initial Release
- 2D occupancy map generation (PGM, PNG, YAML)
- 3D point cloud export (PCD format)
- Octomap generation (binary .bt format)
- Geometry-based ray casting collision detection
- Support for box, cylinder, sphere, and mesh geometries
- Configurable scanning parameters
- ROS 2 service interface
- Command-line Python client
- Nav2 compatibility
