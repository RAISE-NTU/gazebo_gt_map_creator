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

#include "gazebo_gt_map_creator/GazeboGtMapCreator.hpp"

#include <fstream>
#include <iostream>
#include <cmath>
#include <unordered_map>

#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/SemanticLabel.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Line3.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/RayQuery.hh>

#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Sphere.hh>
#include <sdf/Mesh.hh>

#include <gazebo_gt_map_creator/srv/map_request.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/gil.hpp>
#include <boost/gil/extension/io/png.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>

namespace gazebo_gt_map_creator
{

class GazeboGtMapCreator::Implementation
{
public:
  /// ROS node
  rclcpp::Node::SharedPtr ros_node_;

  /// ROS service for map creation
  rclcpp::Service<gazebo_gt_map_creator::srv::MapRequest>::SharedPtr map_service_;

  /// Pointer to the world
  ignition::gazebo::Entity world_entity_;
  
  /// Entity component manager
  ignition::gazebo::EntityComponentManager * ecm_{nullptr};

  /// World name
  std::string world_name_;

  /// Rendering scene for ray queries
  ignition::rendering::ScenePtr scene_{nullptr};
  
  /// Ray query object
  ignition::rendering::RayQueryPtr ray_query_{nullptr};

  /// Collision bounding boxes cache with semantic labels
  std::vector<std::tuple<ignition::math::AxisAlignedBox, ignition::math::Pose3d, uint32_t>> collision_boxes_;

  /// Initialize rendering for ray casting
  bool InitializeRendering();

  /// Build collision boxes from ECM
  void BuildCollisionBoxes();

  /// Service callback for map creation
  void OnMapCreate(
    const std::shared_ptr<gazebo_gt_map_creator::srv::MapRequest::Request> req,
    std::shared_ptr<gazebo_gt_map_creator::srv::MapRequest::Response> res);

  /// Write PGM file
  void pgm_write_view(const std::string & filename, boost::gil::gray8_view_t & view);

  /// Check for ray collision using geometry-based approach
  bool CheckRayCollision(
    const ignition::math::Vector3d & start,
    const ignition::math::Vector3d & end);

  /// Check for ray collision and return semantic label if hit
  bool CheckRayCollisionWithLabel(
    const ignition::math::Vector3d & start,
    const ignition::math::Vector3d & end,
    uint32_t & label);

  /// Check ray intersection with an axis-aligned box
  bool RayBoxIntersection(
    const ignition::math::Vector3d & ray_start,
    const ignition::math::Vector3d & ray_end,
    const ignition::math::AxisAlignedBox & box,
    const ignition::math::Pose3d & box_pose);
};

GazeboGtMapCreator::GazeboGtMapCreator()
: impl_(std::make_unique<Implementation>())
{
}

GazeboGtMapCreator::~GazeboGtMapCreator()
{
}

void GazeboGtMapCreator::Configure(
  const ignition::gazebo::Entity & entity,
  const std::shared_ptr<const sdf::Element> & /*sdf*/,
  ignition::gazebo::EntityComponentManager & ecm,
  ignition::gazebo::EventManager & /*eventMgr*/)
{
  impl_->world_entity_ = entity;
  impl_->ecm_ = &ecm;

  // Get world name
  auto world_name_comp = ecm.Component<ignition::gazebo::components::Name>(entity);
  if (world_name_comp) {
    impl_->world_name_ = world_name_comp->Data();
  }

  // Initialize ROS node
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  impl_->ros_node_ = rclcpp::Node::make_shared("gazebo_gt_map_creator");

  // Create service
  impl_->map_service_ = impl_->ros_node_->create_service<gazebo_gt_map_creator::srv::MapRequest>(
    "/world/save_map",
    std::bind(
      &GazeboGtMapCreator::Implementation::OnMapCreate,
      impl_.get(),
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Gazebo GT Map Creator plugin loaded for world: %s", impl_->world_name_.c_str());
}

void GazeboGtMapCreator::PostUpdate(
  const ignition::gazebo::UpdateInfo & /*info*/,
  const ignition::gazebo::EntityComponentManager & /*ecm*/)
{
  // Spin ROS node to process callbacks
  rclcpp::spin_some(impl_->ros_node_);
}

void GazeboGtMapCreator::Implementation::BuildCollisionBoxes()
{
  collision_boxes_.clear();

  if (!ecm_) {
    RCLCPP_ERROR(ros_node_->get_logger(), "ECM not initialized");
    return;
  }

  // Iterate through all collision entities
  ecm_->Each<ignition::gazebo::components::Collision,
             ignition::gazebo::components::Geometry,
             ignition::gazebo::components::Pose>(
    [&](const ignition::gazebo::Entity & entity,
        const ignition::gazebo::components::Collision *,
        const ignition::gazebo::components::Geometry * geometry,
        const ignition::gazebo::components::Pose * /*local_pose*/) -> bool
    {
      // Get parent link to check if this is a sensor or light
      auto parent = ecm_->Component<ignition::gazebo::components::ParentEntity>(entity);
      if (parent) {
        auto parent_name = ecm_->Component<ignition::gazebo::components::Name>(parent->Data());
        if (parent_name) {
          std::string name = parent_name->Data();
          // Skip sensor links and light sources
          if (name.find("sensor") != std::string::npos ||
              name.find("camera") != std::string::npos ||
              name.find("lidar") != std::string::npos ||
              name.find("light") != std::string::npos ||
              name.find("sun") != std::string::npos) {
            return true;  // Skip this entity
          }
        }
      }
      
      // Get semantic label from parent model
      uint32_t semantic_label = 0;
      auto link_entity = ecm_->Component<ignition::gazebo::components::ParentEntity>(entity);
      if (link_entity) {
        auto model_entity = ecm_->Component<ignition::gazebo::components::ParentEntity>(link_entity->Data());
        if (model_entity) {
          auto label_comp = ecm_->Component<ignition::gazebo::components::SemanticLabel>(model_entity->Data());
          if (label_comp) {
            semantic_label = label_comp->Data();
          }
        }
      }
      
      // Get the geometry
      const sdf::Geometry & geom = geometry->Data();
      
      // Get world pose of collision
      ignition::math::Pose3d world_pose = ignition::gazebo::worldPose(entity, *ecm_);
      
      // Create bounding box based on geometry type
      ignition::math::AxisAlignedBox bbox;
      
      switch (geom.Type()) {
        case sdf::GeometryType::BOX:
        {
          auto box_size = geom.BoxShape()->Size();
          bbox = ignition::math::AxisAlignedBox(
            -box_size / 2.0,
            box_size / 2.0);
          break;
        }
        case sdf::GeometryType::CYLINDER:
        {
          double radius = geom.CylinderShape()->Radius();
          double length = geom.CylinderShape()->Length();
          bbox = ignition::math::AxisAlignedBox(
            ignition::math::Vector3d(-radius, -radius, -length/2.0),
            ignition::math::Vector3d(radius, radius, length/2.0));
          break;
        }
        case sdf::GeometryType::SPHERE:
        {
          double radius = geom.SphereShape()->Radius();
          bbox = ignition::math::AxisAlignedBox(
            ignition::math::Vector3d(-radius, -radius, -radius),
            ignition::math::Vector3d(radius, radius, radius));
          break;
        }
        case sdf::GeometryType::MESH:
        {
          // For meshes, create a conservative bounding box
          // This is a simplified approach
          auto scale = geom.MeshShape()->Scale();
          bbox = ignition::math::AxisAlignedBox(
            ignition::math::Vector3d(-1.0, -1.0, -1.0) * scale,
            ignition::math::Vector3d(1.0, 1.0, 1.0) * scale);
          break;
        }
        case sdf::GeometryType::PLANE:
        {
          // Skip infinite planes (typically ground plane)
          return true;
        }
        default:
          // Use a default small box for unknown types
          bbox = ignition::math::AxisAlignedBox(
            ignition::math::Vector3d(-0.1, -0.1, -0.1),
            ignition::math::Vector3d(0.1, 0.1, 0.1));
          break;
      }
      
      // Skip extremely large objects (likely ground planes modeled as boxes)
      auto size = bbox.Size();
      if (size.X() > 1000 || size.Y() > 1000) {
        return true;
      }
      
      collision_boxes_.push_back({bbox, world_pose, semantic_label});
      
      // Debug output for first few collisions
      if (collision_boxes_.size() <= 5) {
        RCLCPP_INFO(ros_node_->get_logger(), 
          "Collision %zu: Type=%d, Label=%u, BBox=[%.2f,%.2f,%.2f]-[%.2f,%.2f,%.2f], Pose=[%.2f,%.2f,%.2f]",
          collision_boxes_.size(), static_cast<int>(geom.Type()), semantic_label,
          bbox.Min().X(), bbox.Min().Y(), bbox.Min().Z(),
          bbox.Max().X(), bbox.Max().Y(), bbox.Max().Z(),
          world_pose.Pos().X(), world_pose.Pos().Y(), world_pose.Pos().Z());
      }
      
      return true;
    });
  
  RCLCPP_INFO(ros_node_->get_logger(), "Found %zu collision objects in scene", collision_boxes_.size());
}

bool GazeboGtMapCreator::Implementation::RayBoxIntersection(
  const ignition::math::Vector3d & ray_start,
  const ignition::math::Vector3d & ray_end,
  const ignition::math::AxisAlignedBox & box,
  const ignition::math::Pose3d & box_pose)
{
  // Transform ray to box's local coordinate system
  auto inv_pose = box_pose.Inverse();
  auto local_start = inv_pose.Rot() * (ray_start - box_pose.Pos());
  auto local_end = inv_pose.Rot() * (ray_end - box_pose.Pos());
  
  // Ray direction and length
  auto ray_dir = local_end - local_start;
  double ray_length = ray_dir.Length();
  
  if (ray_length < 1e-6) {
    return false;
  }
  
  ray_dir.Normalize();
  
  // Ray-AABB intersection test using slab method
  auto box_min = box.Min();
  auto box_max = box.Max();
  
  double tmin = 0.0;
  double tmax = ray_length;
  
  for (int i = 0; i < 3; ++i) {
    if (std::abs(ray_dir[i]) < 1e-6) {
      // Ray is parallel to slab
      if (local_start[i] < box_min[i] || local_start[i] > box_max[i]) {
        return false;
      }
    } else {
      // Compute intersection t values
      double t1 = (box_min[i] - local_start[i]) / ray_dir[i];
      double t2 = (box_max[i] - local_start[i]) / ray_dir[i];
      
      if (t1 > t2) std::swap(t1, t2);
      
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);
      
      if (tmin > tmax) {
        return false;
      }
    }
  }
  
  // Check if intersection is within ray segment
  return tmin <= ray_length && tmax >= 0.0;
}

bool GazeboGtMapCreator::Implementation::CheckRayCollision(
  const ignition::math::Vector3d & start,
  const ignition::math::Vector3d & end)
{
  // Check collision with all boxes
  for (const auto & [box, pose, label] : collision_boxes_) {
    if (RayBoxIntersection(start, end, box, pose)) {
      return true;
    }
  }
  
  return false;
}

bool GazeboGtMapCreator::Implementation::CheckRayCollisionWithLabel(
  const ignition::math::Vector3d & start,
  const ignition::math::Vector3d & end,
  uint32_t & label)
{
  // Check collision with all boxes and return the label of the first hit
  for (const auto & [box, pose, box_label] : collision_boxes_) {
    if (RayBoxIntersection(start, end, box, pose)) {
      label = box_label;
      return true;
    }
  }
  
  return false;
}

void GazeboGtMapCreator::Implementation::OnMapCreate(
  const std::shared_ptr<gazebo_gt_map_creator::srv::MapRequest::Request> req,
  std::shared_ptr<gazebo_gt_map_creator::srv::MapRequest::Response> res)
{
  RCLCPP_INFO(ros_node_->get_logger(), "Received map creation request");

  // Calculate the size of the cubic area
  float size_x = req->upperleft.x - req->lowerright.x;
  float size_y = req->lowerright.y - req->upperleft.y;  // size_y to be -ve
  float size_z = req->upperleft.z - req->lowerright.z;

  // Check for coordinate validity
  if (size_x <= 0 || size_y >= 0 || size_z <= 0) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Invalid coordinates");
    res->success = false;
    return;
  }

  // Calculate the number of points in each dimension
  int num_points_x = static_cast<int>(std::abs(size_x) / req->resolution) + 1;
  int num_points_y = static_cast<int>(std::abs(size_y) / req->resolution) + 1;
  int num_points_z = static_cast<int>(std::abs(size_z) / req->resolution) + 1;

  // Calculate the step size in each dimension
  float step_x = size_x / num_points_x;
  float step_y = size_y / num_points_y;
  float step_z = size_z / num_points_z;

  int dims = 6;

  if (req->skip_vertical_scan) {
    num_points_z = 2;
    step_z = size_z;
    dims = 4;
  }

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "-----------------\n"
    "Area Corners: (lower right, upper left) (%f, %f, %f), (%f, %f, %f)\n"
    "Area size: %f x %f x %f (WxLxH)\n"
    "Step size: %f, %f, %f (stepx, stepy, stepz)\n"
    "Resolution: (%d, %d, %d) - %f\n"
    "Map Mode: %s\n"
    "-----------------",
    req->lowerright.x, req->lowerright.y, req->lowerright.z,
    req->upperleft.x, req->upperleft.y, req->upperleft.z,
    size_x, size_y, size_z,
    step_x, step_y, step_z,
    num_points_x, num_points_y, num_points_z, req->resolution,
    req->skip_vertical_scan ? "Partial Scan" : "Full Scan");

  // Nav2 expects: 0 = occupied (black), 255 = free (white), 205 = unknown (grey)
  // For proper Nav2 compatibility, use pure black for obstacles
  boost::gil::gray8_pixel_t fill(0);      // Occupied cells = black
  boost::gil::gray8_pixel_t blank(255);   // Free cells = white
  boost::gil::gray8_image_t image(num_points_x, num_points_y);

  // Initially fill all area with empty pixel value
  boost::gil::fill_pixels(image._view, blank);

  // Create point cloud objects - use labeled type if semantic labels are requested
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZL> labeled_cloud;
  
  if (req->capture_semantic_labels) {
    labeled_cloud.width = num_points_x;
    labeled_cloud.height = num_points_y;
    RCLCPP_INFO(ros_node_->get_logger(), "Semantic label capture ENABLED");
  } else {
    cloud.width = num_points_x;
    cloud.height = num_points_y;
    RCLCPP_INFO(ros_node_->get_logger(), "Semantic label capture DISABLED");
  }

  struct PointMask {
    int x, y, z;
  };
  
  PointMask directions[6] = {
    {-1, 0, 0},  // Left
    {1, 0, 0},   // Right
    {0, 1, 0},   // Front
    {0, -1, 0},  // Back
    {0, 0, 1},   // Top
    {0, 0, -1}   // Bottom
  };

  // Build collision boxes from current scene state
  BuildCollisionBoxes();
  
  RCLCPP_INFO(
    ros_node_->get_logger(),
    "Starting map generation with %zu collision objects",
    collision_boxes_.size());

  for (int x = 0; x < num_points_x; ++x) {
    if (x % 10 == 0) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "Percent complete: %.1f%%",
        x * 100.0 / num_points_x);
    }

    double cur_x = req->lowerright.x + x * step_x;
    for (int y = 0; y < num_points_y; ++y) {
      double cur_y = req->upperleft.y + y * step_y;

      ignition::math::Vector3d startV(cur_x, cur_y, req->lowerright.z);
      ignition::math::Vector3d endV(cur_x, cur_y, req->upperleft.z);

      // Check vertical ray for collision
      bool collision_detected = false;
      uint32_t semantic_label = 0;
      
      if (req->capture_semantic_labels) {
        collision_detected = CheckRayCollisionWithLabel(startV, endV, semantic_label);
      } else {
        collision_detected = CheckRayCollision(startV, endV);
      }

      if (collision_detected) {
        image._view(x, y) = fill;
      }

      // Walk in z direction to check each point for any collision to its neighbours
      for (int z = 0; z < num_points_z; ++z) {
        double cur_z = req->lowerright.z + z * step_z;
        ignition::math::Vector3d start(cur_x, cur_y, cur_z);

        // Check for right, left, front, back, top and bottom points for collision
        for (int i = 0; i < dims; ++i) {
          double dx = directions[i].x * step_x * req->range_multiplier;
          double dy = directions[i].y * step_y * req->range_multiplier;
          double dz = directions[i].z * step_z * req->range_multiplier;
          ignition::math::Vector3d end(cur_x + dx, cur_y + dy, cur_z + dz);

          // Perform ray collision check
          if (req->capture_semantic_labels) {
            collision_detected = CheckRayCollisionWithLabel(start, end, semantic_label);
          } else {
            collision_detected = CheckRayCollision(start, end);
          }

          if (collision_detected) {
            if (req->capture_semantic_labels) {
              pcl::PointXYZL point;
              point.x = cur_x;
              point.y = cur_y;
              point.z = cur_z;
              point.label = semantic_label;
              labeled_cloud.push_back(point);
            } else {
              cloud.push_back(pcl::PointXYZ(cur_x, cur_y, cur_z));
            }
            image._view(x, y) = fill;
            break;
          }
        }
      }
    }
  }

  RCLCPP_INFO(ros_node_->get_logger(), "Completed calculations, writing to files");

  if (!req->filename.empty()) {
    // Determine which point cloud to use
    size_t cloud_size = req->capture_semantic_labels ? labeled_cloud.size() : cloud.size();
    
    if (cloud_size > 0) {
      // Save pcd file
      if (req->capture_semantic_labels) {
        pcl::io::savePCDFileASCII(req->filename + ".pcd", labeled_cloud);
        RCLCPP_INFO(ros_node_->get_logger(), 
          "Saved labeled point cloud with %zu points", labeled_cloud.size());
      } else {
        pcl::io::savePCDFileASCII(req->filename + ".pcd", cloud);
        RCLCPP_INFO(ros_node_->get_logger(), 
          "Saved point cloud with %zu points", cloud.size());
      }

      // Save octomap file
      octomap::OcTree octree(req->resolution);
      if (req->capture_semantic_labels) {
        for (const auto & p : labeled_cloud.points) {
          octree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
        }
      } else {
        for (const auto & p : cloud.points) {
          octree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
        }
      }
      octree.updateInnerOccupancy();
      octree.writeBinary(req->filename + ".bt");
    }

    // Save png file
    boost::gil::gray8_view_t view = image._view;
    boost::gil::write_view(req->filename + ".png", view, boost::gil::png_tag());

    // Save pgm file
    pgm_write_view(req->filename, view);

    // Write down yaml file for nav2 usage
    // Extract just the filename (without path) for the image field
    std::string filename_only = req->filename;
    size_t last_slash = filename_only.find_last_of("/\\");
    if (last_slash != std::string::npos) {
      filename_only = filename_only.substr(last_slash + 1);
    }
    
    std::unordered_map<std::string, std::string> yaml_dict;
    yaml_dict["image"] = filename_only + ".pgm";
    yaml_dict["mode"] = "trinary";
    yaml_dict["resolution"] = std::to_string(req->resolution);
    yaml_dict["origin"] = "[" + std::to_string(req->lowerright.x) + std::string(", ") +
      std::to_string(req->lowerright.y) + std::string(", 0.0]");
    yaml_dict["negate"] = "0";
    yaml_dict["occupied_thresh"] = "0.95";
    yaml_dict["free_thresh"] = "0.90";

    std::ofstream outputFile(req->filename + ".yaml");
    if (outputFile.is_open()) {
      for (const auto & pair : yaml_dict) {
        outputFile << pair.first << ": " << pair.second << std::endl;
      }
      outputFile.close();
    } else {
      RCLCPP_ERROR(ros_node_->get_logger(), "Unable to open yaml file for writing.");
    }

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Output location: %s[.pcd, .bt, .pgm, .png, .yaml]",
      req->filename.c_str());
  }

  res->success = true;
}

void GazeboGtMapCreator::Implementation::pgm_write_view(
  const std::string & filename,
  boost::gil::gray8_view_t & view)
{
  // Write image to pgm file
  int h = view.height();
  int w = view.width();

  std::ofstream ofs;
  ofs.open(filename + ".pgm");
  ofs << "P2" << '\n';           // grayscale
  ofs << w << ' ' << h << '\n';  // width and height
  ofs << 255 << '\n';            // max value
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      ofs << static_cast<int>(view(x, y)[0]) << ' ';
    }
    ofs << '\n';
  }
  ofs.close();
}

}  // namespace gazebo_gt_map_creator

// Register the plugin
IGNITION_ADD_PLUGIN(
  gazebo_gt_map_creator::GazeboGtMapCreator,
  ignition::gazebo::System,
  gazebo_gt_map_creator::GazeboGtMapCreator::ISystemConfigure,
  gazebo_gt_map_creator::GazeboGtMapCreator::ISystemPostUpdate)
