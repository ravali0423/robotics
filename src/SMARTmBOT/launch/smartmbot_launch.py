import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    smartmbot_gazebo_dir = get_package_share_directory('SMARTmBOT_gazebo')
    
    # Include Gazebo simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(smartmbot_gazebo_dir, 'launch', 'smartmbot_simulation.launch.py')
        ])
    )
    
    # SMARTmBOT control node
    smartmbot_node = Node(
        package='SMARTmBOT',
        executable='smartmbot_node',
        name='smartmbot_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo_launch,
        smartmbot_node,
    ])