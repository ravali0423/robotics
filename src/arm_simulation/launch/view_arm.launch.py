import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the UR5 URDF from the official ur_description package with proper parameters
    robot_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([
            FindPackageShare('ur_description'),
            'urdf',
            'ur.urdf.xacro'
        ]),
        ' ur_type:=ur5',
        ' name:=ur',
        ' safety_limits:=true',
        ' safety_pos_margin:=0.15',
        ' safety_k_position:=20',
        ' prefix:=""'
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    # RViz config file - using the working official config
    rviz_config_file = os.path.join(get_package_share_directory('arm_simulation'), 'rviz', 'ur5_arm_working.rviz')

    return LaunchDescription([
        # Robot State Publisher - with explicit frequency
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
                'publish_frequency': 30.0
            }]
        ),
        
        # Joint State Publisher GUI with enhanced parameters
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'rate': 30,
                'dependent_joints': {},
                'zeros': {},
                'robot_description': robot_description
            }],
            remappings=[
                ('/joint_states', '/joint_states')
            ]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{
                'use_sim_time': False
            }]
        )
    ])
