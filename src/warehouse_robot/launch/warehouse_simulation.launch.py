#!/usr/bin/env python3
"""
Simple warehouse robot simulation using iRobot Create3.
The robot will be spawned at height and drop onto the ground plane.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    warehouse_robot_pkg = FindPackageShare('warehouse_robot')
    irobot_create_gz_bringup_pkg = FindPackageShare('irobot_create_gz_bringup')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='warehouse_robot',
        description='Name of the robot'
    )
    
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='X position to spawn robot'
    )
    
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Y position to spawn robot'
    )
    
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='2.0',  # Start 2 meters high for drop test
        description='Z position to spawn robot (height for drop test)'
    )

    # Launch the warehouse world with Create3 robot
    create3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                irobot_create_gz_bringup_pkg,
                'launch',
                'create3_gz.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'x': LaunchConfiguration('spawn_x'),
            'y': LaunchConfiguration('spawn_y'),
            'z': LaunchConfiguration('spawn_z'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # Override world file with our warehouse world
            'world_file': PathJoinSubstitution([
                warehouse_robot_pkg,
                'worlds',
                'warehouse_world.sdf'
            ])
        }.items()
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        robot_name_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        
        # Launch Create3 simulation
        create3_launch
    ])