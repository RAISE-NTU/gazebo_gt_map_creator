# Changelog

All notable changes to the gazebo_gt_map_creator project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **Semantic Label Support**: Capture semantic labels from Gazebo's Label System plugin
  - New `capture_semantic_labels` parameter in MapRequest service
  - Support for `PointXYZL` point cloud format (points with labels)
  - Automatic label detection from models with `gz-sim-label-system` plugin
  - New `--capture-labels` / `-c` flag in save_map.py script
  - Comprehensive documentation in SEMANTIC_LABELS.md
  - Example world with labeled models (worlds/example_world.sdf)

### Changed
- Updated collision box storage to include semantic label information
- Modified point cloud generation to conditionally use labeled or unlabeled format
- Enhanced debug output to show detected semantic labels
- Updated README.md with semantic label feature overview

### Technical Details
- Added `ignition::gazebo::components::SemanticLabel` component query
- Collision boxes now stored as tuples: `(bbox, pose, label)`
- New `CheckRayCollisionWithLabel()` method for label-aware collision detection
- Conditional point cloud type selection based on `capture_semantic_labels` flag
- Labels retrieved from parent model entity during collision box building

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
