# Example Launch Files for Ignition Map Creator

This directory contains example launch files demonstrating how to integrate the map creator with your projects.

## Usage with Python Launch Files

If you want to launch Ignition Gazebo with the map creator from a ROS 2 launch file:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to your world file
    world_file = os.path.join(
        get_package_share_directory('ignition_map_creator'),
        'worlds',
        'example_world.sdf'
    )
    
    return LaunchDescription([
        # Launch Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file],
            output='screen'
        ),
        
        # Optional: Launch ROS-Gazebo bridge for additional topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen'
        ),
    ])
```

## Integrating with Nav2

After generating a map, launch Nav2:

```bash
# Generate the map first
ros2 run ignition_map_creator save_map.py /tmp/my_map

# Launch Nav2 with the generated map
ros2 launch nav2_bringup bringup_launch.py \
  map:=/tmp/my_map.yaml \
  use_sim_time:=true
```

## Automated Map Generation

You can create a launch file that starts the simulation and automatically generates a map:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start simulation
        ExecuteProcess(
            cmd=['ign', 'gazebo', 'your_world.sdf'],
            output='screen'
        ),
        
        # Wait for simulation to start, then generate map
        TimerAction(
            period=5.0,  # Wait 5 seconds
            actions=[
                Node(
                    package='ignition_map_creator',
                    executable='save_map.py',
                    output='screen'
                )
            ]
        ),
    ])
```

For more examples, see the ROS 2 launch documentation.
