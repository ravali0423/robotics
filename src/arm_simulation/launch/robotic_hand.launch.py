import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the URDF file path
    robot_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([
            FindPackageShare('arm_simulation'),
            'urdf',
            'robotic_hand.urdf.xacro'
        ])
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    # RViz config file
    rviz_config_file = os.path.join(
        get_package_share_directory('arm_simulation'), 
        'rviz', 
        'robotic_hand.rviz'
    )

    nodes_to_launch = [
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),
        
        # Enhanced Hand Controller Node
        Node(
            package='arm_simulation',
            executable='enhanced_hand_controller.py',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        )
    ]
    
    # Add RViz with config if it exists, otherwise without config
    if os.path.exists(rviz_config_file):
        nodes_to_launch.append(
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file]
            )
        )
    else:
        nodes_to_launch.append(
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen'
            )
        )

    return LaunchDescription(nodes_to_launch)