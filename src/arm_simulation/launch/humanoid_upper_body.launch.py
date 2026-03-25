#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare('arm_simulation')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_share, 'urdf', 'humanoid_upper_body.urdf.xacro'])
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    rviz_config_file = os.path.join(
        get_package_share_directory('arm_simulation'),
        'rviz',
        'humanoid_upper_body.rviz'
    )

    return LaunchDescription([

        # Robot state publisher for the humanoid upper body URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),

        # Dual Hand Coordinator - splits /finger_count and /letter_command
        Node(
            package='arm_simulation',
            executable='dual_hand_coordinator.py',
            name='dual_hand_coordinator',
            output='screen'
        ),

        # Left hand controller
        Node(
            package='arm_simulation',
            executable='enhanced_hand_controller.py',
            name='left_hand_controller',
            output='screen',
            parameters=[{
                'hand_side': 'left',
                'use_sim_time': False
            }]
        ),

        # Right hand controller
        Node(
            package='arm_simulation',
            executable='enhanced_hand_controller.py',
            name='right_hand_controller',
            output='screen',
            parameters=[{
                'hand_side': 'right',
                'use_sim_time': False
            }]
        ),

        # RViz2 with humanoid configuration
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
