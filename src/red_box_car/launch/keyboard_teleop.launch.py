#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch file to test keyboard teleop independently"""
    
    # Keyboard Teleop Node
    keyboard_teleop = Node(
        package='red_box_car',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(keyboard_teleop)
    
    return ld