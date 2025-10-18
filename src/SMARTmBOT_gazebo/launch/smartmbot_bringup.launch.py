import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    smartmbot_gazebo_dir = get_package_share_directory('SMARTmBOT_gazebo')
    
    # SMARTmBOT node
    smartmbot_node = Node(
        package='SMARTmBOT',
        executable='smartmbot_node',
        name='smartmbot_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Robot teleop
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        smartmbot_node,
        teleop_node,
    ])