#!/usr/bin/env python3
"""
ROS 2 client for creating maps from Ignition Gazebo simulations.

This script provides a command-line interface to the map creation service
provided by the gazebo_gt_map_creator plugin. It generates 2D occupancy maps,
3D point clouds, and octomaps from simulated environments.

Usage:
    # Use default parameters (interactive mode)
    ros2 run gazebo_gt_map_creator save_map.py

    # Specify output file
    ros2 run gazebo_gt_map_creator save_map.py /path/to/output/map

    # Specify file and resolution
    ros2 run gazebo_gt_map_creator save_map.py /path/to/output/map 0.05

    # Example with full path
    ros2 run gazebo_gt_map_creator save_map.py ~/maps/my_world_map 0.1
"""

import sys
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from gazebo_gt_map_creator.srv import MapRequest


class MapCreatorClient(Node):
    """ROS 2 client node for calling the map creation service."""

    def __init__(self):
        super().__init__('map_creator_client')
        self.client = self.create_client(MapRequest, '/world/save_map')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /world/save_map service...')

    def create_map(self, 
                   upperleft, 
                   lowerright, 
                   resolution=0.05,
                   threshold_2d=50,
                   filename='map',
                   skip_vertical_scan=False,
                   range_multiplier=1.5):
        """
        Create a map from the simulation.
        
        Args:
            upperleft: Tuple (x, y, z) for upper left corner of the map area
            lowerright: Tuple (x, y, z) for lower right corner of the map area
            resolution: Map resolution in meters (default: 0.05)
            threshold_2d: Threshold for 2D occupancy, 0-100 (default: 50)
            filename: Output filename without extension (default: 'map')
            skip_vertical_scan: Skip 3D scanning for faster 2D maps (default: False)
            range_multiplier: Multiplier for ray casting range (default: 1.5)
        
        Returns:
            Future object for the async service call
        """
        request = MapRequest.Request()
        
        # Set map area corners
        request.upperleft = Point(x=upperleft[0], y=upperleft[1], z=upperleft[2])
        request.lowerright = Point(x=lowerright[0], y=lowerright[1], z=lowerright[2])
        
        # Set parameters
        request.resolution = resolution
        request.threshold_2d = threshold_2d
        request.filename = filename
        request.skip_vertical_scan = skip_vertical_scan
        request.range_multiplier = range_multiplier
        
        self.get_logger().info(f'Creating map with bounds: {upperleft} to {lowerright}')
        self.get_logger().info(f'Resolution: {resolution}m, Output: {filename}')
        self.get_logger().info(f'Skip vertical scan: {skip_vertical_scan}')
        
        future = self.client.call_async(request)
        return future


def main(args=None):
    """Main function to run the map creation client."""
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Create maps from Ignition Gazebo simulations',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ros2 run gazebo_gt_map_creator save_map.py --filename $PWD/map --upper-left 6 6 2 --lower-right -6 -6 0 --resolution 0.1
  ros2 run gazebo_gt_map_creator save_map.py -f ~/maps/maze -u 5 5 2 -l -5 -5 0 -r 0.05
  ros2 run gazebo_gt_map_creator save_map.py --filename /tmp/map --upper-left -10 10 3 --lower-right 10 -10 0
        """)
    
    parser.add_argument('--filename', '-f',
                        type=str,
                        default='/tmp/map',
                        help='Output filename without extension (default: /tmp/map)')
    
    parser.add_argument('--upper-left', '-u',
                        type=float,
                        nargs=3,
                        metavar=('X', 'Y', 'Z'),
                        default=[-6.0, 6.0, 2.0],
                        help='Upper left corner coordinates (x y z) (default: -6 6 2)')
    
    parser.add_argument('--lower-right', '-l',
                        type=float,
                        nargs=3,
                        metavar=('X', 'Y', 'Z'),
                        default=[6.0, -6.0, 0.0],
                        help='Lower right corner coordinates (x y z) (default: 6 -6 0)')
    
    parser.add_argument('--resolution', '-r',
                        type=float,
                        default=0.05,
                        help='Map resolution in meters per pixel (default: 0.05)')
    
    parser.add_argument('--threshold', '-t',
                        type=int,
                        default=50,
                        help='Occupancy threshold 0-100 (default: 50, Note: obstacles are always black for Nav2 compatibility)')
    
    parser.add_argument('--skip-vertical-scan', '-s',
                        action='store_true',
                        help='Skip 3D scanning for faster 2D-only maps')
    
    parser.add_argument('--range-multiplier', '-m',
                        type=float,
                        default=1.5,
                        help='Ray casting range multiplier (default: 1.5)')
    
    # Parse arguments (filtering out ROS args)
    parsed_args = parser.parse_args()
    
    # Extract values
    filename = parsed_args.filename
    upperleft = tuple(parsed_args.upper_left)
    lowerright = tuple(parsed_args.lower_right)
    resolution = parsed_args.resolution
    threshold = parsed_args.threshold
    skip_vertical = parsed_args.skip_vertical_scan
    range_mult = parsed_args.range_multiplier
    
    # Initialize ROS
    rclpy.init(args=args)
    
    print("=" * 60)
    print("Ignition Gazebo Map Creator Client")
    print("=" * 60)
    print(f"Output filename: {filename}")
    print(f"Resolution: {resolution} m/pixel")
    print(f"Upper left:  ({upperleft[0]}, {upperleft[1]}, {upperleft[2]})")
    print(f"Lower right: ({lowerright[0]}, {lowerright[1]}, {lowerright[2]})")
    print(f"Threshold: {threshold}")
    print(f"Skip vertical scan: {skip_vertical}")
    print(f"Range multiplier: {range_mult}")
    print("=" * 60)
    
    client = MapCreatorClient()
    
    future = client.create_map(
        upperleft=upperleft,
        lowerright=lowerright,
        resolution=resolution,
        threshold_2d=threshold,
        filename=filename,
        skip_vertical_scan=skip_vertical,
        range_multiplier=range_mult
    )
    
    rclpy.spin_until_future_complete(client, future)
    
    if future.result() is not None:
        response = future.result()
        if response.success:
            client.get_logger().info('Map created successfully!')
            client.get_logger().info(f'Output files: {filename}.[pgm, png, yaml, pcd, bt]')
            print("\n" + "=" * 60)
            print("SUCCESS! Map files created:")
            print(f"  - {filename}.pgm   (Grayscale map for Nav2)")
            print(f"  - {filename}.png   (Color visualization)")
            print(f"  - {filename}.yaml  (Nav2 map metadata)")
            print(f"  - {filename}.pcd   (Point cloud)")
            print(f"  - {filename}.bt    (Binary octomap)")
            print("=" * 60)
        else:
            client.get_logger().error('Map creation failed!')
            print("\nERROR: Map creation failed. Check the coordinates.")
    else:
        client.get_logger().error('Service call failed!')
        print("\nERROR: Service call failed. Is the plugin loaded?")
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
