import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the UR5 URDF from the official ur_description package
    robot_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([
            FindPackageShare('ur_description'),
            'urdf',
            'ur.urdf.xacro'
        ]),
        ' name:=ur5',
        ' ur_type:=ur5'
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    # RViz config file
    rviz_config_file = os.path.join(get_package_share_directory('arm_simulation'), 'rviz', 'ur5_arm_working.rviz')

    return LaunchDescription([
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
        
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
