#!/usr/bin/env python3
"""
Publish a PCD file as a ROS 2 PointCloud2 message for visualization in RViz2.

Usage:
    ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py /path/to/file.pcd
    ros2 run gazebo_gt_map_creator publish_pcd_to_rviz.py /path/to/file.pcd --frame map --rate 1
"""

import sys
import argparse
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class PCDPublisher(Node):
    """Node that publishes a PCD file as PointCloud2 messages."""

    def __init__(self, pcd_file, frame_id='map', rate=1.0):
        super().__init__('pcd_publisher')
        
        self.frame_id = frame_id
        self.pcd_file = pcd_file
        
        # Create publisher
        self.publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)
        
        # Load PCD file
        self.points = self.load_pcd(pcd_file)
        
        if not self.points:
            self.get_logger().error(f'Failed to load PCD file: {pcd_file}')
            return
        
        self.get_logger().info(f'Loaded {len(self.points)} points from {pcd_file}')
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / rate, self.publish_pointcloud)
        
        self.get_logger().info(f'Publishing point cloud on topic "point_cloud" at {rate} Hz')
        self.get_logger().info(f'Frame: {frame_id}')
        self.get_logger().info('Visualize in RViz2: Add PointCloud2 display, set topic to /point_cloud')

    def load_pcd(self, filename):
        """Load a PCD file and return list of (x, y, z, r, g, b) tuples."""
        points = []
        
        try:
            with open(filename, 'r') as f:
                lines = f.readlines()
            
            # Parse header
            header = {}
            data_start = 0
            for i, line in enumerate(lines):
                line = line.strip()
                if line.startswith('DATA'):
                    data_start = i + 1
                    break
                if ' ' in line or '\t' in line:
                    parts = line.split(None, 1)
                    if len(parts) == 2:
                        header[parts[0]] = parts[1]
            
            # Check if it's RGB point cloud
            fields = header.get('FIELDS', '').split()
            has_rgb_packed = 'rgb' in fields  # Single packed RGB field
            has_rgb_separate = 'r' in fields and 'g' in fields and 'b' in fields
            
            self.get_logger().info(f'PCD Fields: {fields}')
            self.get_logger().info(f'RGB packed: {has_rgb_packed}, RGB separate: {has_rgb_separate}')
            
            # Parse data
            for line in lines[data_start:]:
                line = line.strip()
                if not line:
                    continue
                
                parts = line.split()
                if len(parts) >= 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    
                    # Extract RGB if available
                    if has_rgb_packed and len(parts) >= 4:
                        # RGB is packed as single uint32
                        rgb_packed = int(float(parts[3]))
                        r = (rgb_packed >> 16) & 0xFF
                        g = (rgb_packed >> 8) & 0xFF
                        b = rgb_packed & 0xFF
                    elif has_rgb_separate and len(parts) >= 6:
                        # Separate R G B fields
                        r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                    else:
                        # Default white color
                        r, g, b = 255, 255, 255
                    
                    points.append((x, y, z, r, g, b))
            
            return points
            
        except Exception as e:
            self.get_logger().error(f'Error loading PCD file: {e}')
            return []

    def publish_pointcloud(self):
        """Publish the point cloud."""
        if not self.points:
            return
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16  # 4 bytes * 4 fields
        msg.is_dense = True
        
        # Pack point data
        buffer = []
        for x, y, z, r, g, b in self.points:
            # Pack RGB as single uint32
            rgb = (r << 16) | (g << 8) | b
            buffer.append(struct.pack('fffI', x, y, z, rgb))
        
        msg.data = b''.join(buffer)
        msg.row_step = msg.point_step * len(self.points)
        msg.width = len(self.points)
        msg.height = 1
        
        self.publisher.publish(msg)


def main(args=None):
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Publish PCD file to RViz2 as PointCloud2',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    
    parser.add_argument('pcd_file',
                        type=str,
                        help='Path to PCD file')
    
    parser.add_argument('--frame', '-f',
                        type=str,
                        default='map',
                        help='Frame ID for the point cloud (default: map)')
    
    parser.add_argument('--rate', '-r',
                        type=float,
                        default=1.0,
                        help='Publishing rate in Hz (default: 1.0)')
    
    parsed_args = parser.parse_args()
    
    # Initialize ROS
    rclpy.init(args=args)
    
    print("=" * 60)
    print("PCD to RViz2 Publisher")
    print("=" * 60)
    print(f"PCD file: {parsed_args.pcd_file}")
    print(f"Frame ID: {parsed_args.frame}")
    print(f"Rate: {parsed_args.rate} Hz")
    print("=" * 60)
    print("\nTo visualize:")
    print("  1. Run: rviz2")
    print("  2. Set Fixed Frame to:", parsed_args.frame)
    print("  3. Add -> PointCloud2")
    print("  4. Set Topic to: /point_cloud")
    print("  5. Set Color Transformer to: RGB8")
    print("=" * 60)
    
    node = PCDPublisher(parsed_args.pcd_file, parsed_args.frame, parsed_args.rate)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
