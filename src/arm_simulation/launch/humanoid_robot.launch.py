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
        PathJoinSubstitution([pkg_share, 'urdf', 'humanoid_robot.urdf.xacro'])
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    rviz_config_file = os.path.join(
        get_package_share_directory('arm_simulation'),
        'rviz',
        'humanoid_robot.rviz'
    )

    return LaunchDescription([

        # Publishes /robot_description and tf transforms for all links
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),

        # Body controller — publishes 10 body joint states at 10 Hz
        Node(
            package='arm_simulation',
            executable='body_controller.py',
            name='body_controller',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # Left hand controller — publishes 14 left finger joint states
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

        # Right hand controller — publishes 14 right finger joint states
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

        # Dual-hand coordinator — handles /finger_count → /left + /right splits
        Node(
            package='arm_simulation',
            executable='dual_hand_coordinator.py',
            name='dual_hand_coordinator',
            output='screen'
        ),

        # Sign coordinator — listens to /sign_command, drives body + hands
        Node(
            package='arm_simulation',
            executable='humanoid_sign_coordinator.py',
            name='humanoid_sign_coordinator',
            output='screen'
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
