#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_red_box_car = FindPackageShare(package='red_box_car').find('red_box_car')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_red_box_car, 'urdf', 'red_box_car_simple.urdf')
    
    # Path to RViz config file
    rviz_config_file = os.path.join(pkg_red_box_car, 'config', 'red_box_car.rviz')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Joint state publisher GUI (for manual control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Control nodes
    vision_node = Node(
        package='red_box_car',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    navigation_controller = Node(
        package='red_box_car',
        executable='navigation_controller',
        name='navigation_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    pickup_controller = Node(
        package='red_box_car',
        executable='pickup_controller',
        name='pickup_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    state_machine = Node(
        package='red_box_car',
        executable='state_machine',
        name='state_machine',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz)
    ld.add_action(vision_node)
    ld.add_action(navigation_controller)
    ld.add_action(pickup_controller)
    ld.add_action(state_machine)
    
    return ld