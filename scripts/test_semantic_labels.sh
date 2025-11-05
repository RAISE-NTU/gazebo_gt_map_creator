#!/bin/bash

# Example script to test semantic label capture
# This demonstrates the complete workflow

set -e

echo "========================================"
echo "Semantic Label Capture Test Script"
echo "========================================"

# Configuration
WORLD_FILE="$(pwd)/worlds/example_world.sdf"
OUTPUT_DIR="/tmp/gazebo_maps"
MAP_NAME="labeled_test_map"

echo ""
echo "Step 1: Creating output directory"
mkdir -p "$OUTPUT_DIR"
echo "  Output directory: $OUTPUT_DIR"

echo ""
echo "Step 2: Launch Gazebo with example world"
echo "  World file: $WORLD_FILE"
echo "  Please run in a separate terminal:"
echo "  ign gazebo $WORLD_FILE"
echo ""
read -p "Press Enter once Gazebo is running..."

echo ""
echo "Step 3: Generate map WITHOUT semantic labels"
echo "  This will create standard PointXYZ point cloud"
ros2 run gazebo_gt_map_creator save_map.py \
  --filename "$OUTPUT_DIR/${MAP_NAME}_standard" \
  --upper-left -6 6 2 \
  --lower-right 6 -6 0 \
  --resolution 0.05 \
  --skip-vertical-scan

echo ""
echo "Step 4: Generate map WITH semantic labels"
echo "  This will create labeled PointXYZL point cloud"
ros2 run gazebo_gt_map_creator save_map.py \
  --filename "$OUTPUT_DIR/${MAP_NAME}_labeled" \
  --upper-left -6 6 2 \
  --lower-right 6 -6 0 \
  --resolution 0.05 \
  --skip-vertical-scan \
  --capture-labels

echo ""
echo "========================================"
echo "Test Complete!"
echo "========================================"
echo ""
echo "Output files:"
echo "Standard (no labels):"
ls -lh "$OUTPUT_DIR/${MAP_NAME}_standard".*
echo ""
echo "Labeled (with semantic labels):"
ls -lh "$OUTPUT_DIR/${MAP_NAME}_labeled".*
echo ""
echo "To view the maps:"
echo "  eog $OUTPUT_DIR/${MAP_NAME}_standard.png"
echo "  eog $OUTPUT_DIR/${MAP_NAME}_labeled.png"
echo ""
echo "To inspect the point clouds:"
echo "  pcl_viewer $OUTPUT_DIR/${MAP_NAME}_standard.pcd"
echo "  pcl_viewer $OUTPUT_DIR/${MAP_NAME}_labeled.pcd"
echo ""
echo "Compare file sizes:"
echo "  Standard PCD: $(stat --format='%s bytes' $OUTPUT_DIR/${MAP_NAME}_standard.pcd)"
echo "  Labeled PCD:  $(stat --format='%s bytes' $OUTPUT_DIR/${MAP_NAME}_labeled.pcd)"
echo "  (Labeled files are ~33% larger due to the 4-byte label field)"
echo ""
