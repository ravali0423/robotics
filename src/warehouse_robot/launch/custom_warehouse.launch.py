#!/usr/bin/env python3
"""
Custom warehouse simulation using our warehouse world and diff_drive car.
Generates random start/destination waypoints and spawns car at start position.
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def read_waypoints():
    """Read generated waypoints from file."""
    waypoints_file = "/home/ravali/ros2_ws/src/warehouse_robot/scripts/waypoints.txt"
    waypoints = {}
    
    try:
        with open(waypoints_file, 'r') as f:
            for line in f:
                key, value = line.strip().split('=')
                waypoints[key] = float(value)
        
        print("📍 Waypoints loaded:", waypoints)
        return waypoints
    except Exception as e:
        print(f"Warning: Could not read waypoints file: {e}")
        # Return default values
        return {
            'START_X': 0.0,
            'START_Y': 0.0,
            'DEST_X': 5.0,
            'DEST_Y': 5.0
        }


def generate_launch_description():
    # Read existing waypoints (don't generate new ones)
    print("📍 Reading existing waypoints...")
    
    # Read the existing waypoints
    waypoints = read_waypoints()
    spawn_x = waypoints.get('START_X', 0.0)
    spawn_y = waypoints.get('START_Y', 0.0)
    
    print(f"🚗 Car will spawn at start position: ({spawn_x:.2f}, {spawn_y:.2f})")
    
    # Package paths
    warehouse_robot_pkg = FindPackageShare('warehouse_robot')
    
    # Launch arguments (with dynamic defaults from waypoints)
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value=str(spawn_x),
        description='X position to spawn robot (start waypoint)'
    )
    
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value=str(spawn_y),
        description='Y position to spawn robot (start waypoint)'
    )
    
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='5.0',
        description='Height to spawn robot for drop test'
    )

    # Custom warehouse world path
    warehouse_world_path = PathJoinSubstitution([
        warehouse_robot_pkg,
        'worlds',
        'warehouse_world.sdf'
    ])

    # Start Gazebo with our custom world
    start_gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            warehouse_world_path,
            '-v', '4'
        ],
        output='screen'
    )

    # Start physics simulation (unpause)
    start_physics = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to load
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/warehouse_world/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', 'pause: false'
                ],
                output='screen'
            )
        ]
    )

    # Automatically spawn the car after Gazebo loads
    spawn_robot = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to fully load
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/warehouse_world/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', 
                    'sdf_filename: "/home/ravali/ros2_ws/src/warehouse_robot/models/diff_drive/model.sdf", '
                    'name: "warehouse_car", '
                    'pose: {'
                    '  position: {'
                    f'    x: {spawn_x}, '
                    f'    y: {spawn_y}, '
                    '    z: 5.0'
                    '  }'
                    '}'
                ],
                output='screen'
            )
        ]
    )

    # Bridge for car control (connects car to ROS2 topics)
    car_bridge = TimerAction(
        period=7.0,  # Start bridge after car is spawned
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/model/warehouse_car/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/warehouse_car/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        start_gazebo,
        start_physics,
        spawn_robot,
        car_bridge
    ])