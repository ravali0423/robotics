#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths
    pkg_name = 'red_box_car'
    urdf_file = 'red_box_car_simple.urdf'
    rviz_config_file = 'red_box_car.rviz'
    
    # Get the path to the URDF file
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        urdf_file
    )
    
    # Read the URDF file content
    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    # Get the path to RViz config file
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        rviz_config_file
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Joint State Publisher Node (for static joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{
            'use_sim_time': False
        }]
    )

    # Camera Simulator Node
    camera_simulator = Node(
        package='red_box_car',
        executable='camera_simulator',
        name='camera_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the nodes to the launch description
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)
    ld.add_action(camera_simulator)
    
    return ld