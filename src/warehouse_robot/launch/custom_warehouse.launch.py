#!/usr/bin/env python3
"""
Custom warehouse simulation using our warehouse world and diff_drive car.
Car automatically spawns at center and falls from height.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    warehouse_robot_pkg = FindPackageShare('warehouse_robot')
    
    # Launch arguments
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='X position to spawn robot (center of warehouse)'
    )
    
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Y position to spawn robot (center of warehouse)'
    )
    
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='3.0',
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
                    '    x: 0.0, '
                    '    y: 0.0, '
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
                    '/diff_drive/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/diff_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
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