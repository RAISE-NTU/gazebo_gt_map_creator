cmake_minimum_required(VERSION 3.5)

project(gazebo_gt_map_creator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-rendering6 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(sdformat12 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(octomap REQUIRED)

# Find PNG library for boost::gil
find_package(PNG REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MapRequest.srv"
  DEPENDENCIES geometry_msgs
)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# Build the GazeboGtMapCreator plugin
add_library(gazebo_gt_map_creator_system SHARED
  src/GazeboGtMapCreator.cpp
)

target_link_libraries(gazebo_gt_map_creator_system
  ignition-gazebo6::core
  ignition-plugin1::core
  ignition-rendering6::core
  ignition-math6::core
  sdformat12::sdformat12
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PNG_LIBRARIES}
)

ament_target_dependencies(gazebo_gt_map_creator_system
  rclcpp
  geometry_msgs
)

# Ensure service is generated before building the plugin
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gazebo_gt_map_creator_system "${cpp_typesupport_target}")

# Install the plugin library
install(TARGETS gazebo_gt_map_creator_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create symlink for backward compatibility
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \
  libgazebo_gt_map_creator_system.so \
  \${CMAKE_INSTALL_PREFIX}/lib/libignition_map_creator_system.so)")

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install Python scripts
install(PROGRAMS
  scripts/save_map.py
  scripts/publish_pcd_to_rviz.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install RViz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install resource directory for package indexing
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Setup the environmental hooks for the Gazebo system plugin path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_system_plugin_path.dsv.in")

# Export dependencies for downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
