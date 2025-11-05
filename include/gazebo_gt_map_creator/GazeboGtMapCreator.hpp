// Copyright (c) 2025.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GAZEBO_GT_MAP_CREATOR__GAZEBOGTMAPCREATOR_HPP_
#define GAZEBO_GT_MAP_CREATOR__GAZEBOGTMAPCREATOR_HPP_

#include <memory>

#include <ignition/gazebo/System.hh>

namespace gazebo_gt_map_creator
{

/// \brief Plugin that creates 2D and 3D maps from Ignition Gazebo simulations
/// 
/// This plugin provides a ROS 2 service to generate occupancy maps (2D), 
/// point clouds (PCD), and octomaps (OctoMap format) from the simulated world.
///
/// ## Usage
/// Add this plugin to your world SDF file:
/// ```xml
/// <world name="your_world">
///   <plugin
///     filename="libgazebo_gt_map_creator_system.so"
///     name="gazebo_gt_map_creator::GazeboGtMapCreator">
///   </plugin>
///   <!-- ... rest of your world ... -->
/// </world>
/// ```
///
/// ## ROS 2 Service
/// * `/world/save_map` (gazebo_map_creator_interface/srv/MapRequest)
///   - Creates maps based on the simulation environment
///   - Parameters:
///     - upperleft: Upper left corner (x, y, z) of the map area
///     - lowerright: Lower right corner (x, y, z) of the map area
///     - resolution: Map resolution in meters (default: 0.05)
///     - threshold_2d: Occupancy threshold 0-100 (default: 50)
///     - filename: Output filename without extension
///     - skip_vertical_scan: Skip 3D scan for faster 2D maps (default: false)
///     - range_multiplier: Ray casting range multiplier (default: 1.5)
///
/// ## Output Formats
/// The plugin generates multiple file formats:
/// - .pgm: Grayscale occupancy map (for ROS 2 Nav2)
/// - .png: Color occupancy map visualization
/// - .yaml: Map metadata for Nav2 map_server
/// - .pcd: Point cloud (ASCII format)
/// - .bt: Binary octomap
///
/// ## Features
/// - Geometry-based ray collision detection for accuracy
/// - Support for box, cylinder, sphere, and mesh geometries
/// - Configurable scanning resolution and area
/// - Optional 3D scanning for point cloud generation
/// - Automatic filtering of sensors and lights
///
/// ## Example Python Client
/// See scripts/save_map.py for a complete example of calling the service.
class GazeboGtMapCreator :
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate,
  public ignition::gazebo::ISystemPostUpdate
{
public:
  /// \brief Constructor
  GazeboGtMapCreator();

  /// \brief Destructor
  ~GazeboGtMapCreator() override;

  /// Documentation inherited
  void Configure(
    const ignition::gazebo::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager & ecm,
    ignition::gazebo::EventManager & eventMgr) override;

  /// Documentation inherited
  void PreUpdate(
    const ignition::gazebo::UpdateInfo & info,
    ignition::gazebo::EntityComponentManager & ecm) override;

  /// Documentation inherited
  void PostUpdate(
    const ignition::gazebo::UpdateInfo & info,
    const ignition::gazebo::EntityComponentManager & ecm) override;

private:
  /// \brief Private implementation
  class Implementation;
  std::unique_ptr<Implementation> impl_;
};

}  // namespace gazebo_gt_map_creator

#endif  // GAZEBO_GT_MAP_CREATOR__GAZEBOGTMAPCREATOR_HPP_
