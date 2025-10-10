#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths
    pkg_name = 'red_box_car'
    car_urdf_file = 'red_box_car_simple.urdf'
    world_file = 'car_world.sdf'
    
    # Get the path to the URDF and world files
    car_urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        car_urdf_file
    )
    
    world_path = os.path.join(
        get_package_share_directory(pkg_name),
        'worlds',
        world_file
    )
    
    # Read the car URDF file content
    with open(car_urdf_path, 'r') as file:
        car_robot_description = file.read()

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Start Gazebo with our custom world
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-s', '--headless-rendering', '-r', world_path],
        output='screen'
    )
    
    # Start Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-g'],
        output='screen'
    )

    # Robot State Publisher Node for Car only
    car_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='car_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': car_robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn car entity at a height so it falls onto the ground plane
    spawn_car = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', car_urdf_path, '-name', 'red_box_car', '-z', '2.0'],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # ROS-Gazebo Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Camera Simulator Node
    camera_simulator = Node(
        package='red_box_car',
        executable='camera_simulator',
        name='camera_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Red Box Spawner Node
    red_box_spawner = Node(
        package='red_box_car',
        executable='red_box_spawner',
        name='red_box_spawner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Red Box Tracker Node
    red_box_tracker = Node(
        package='red_box_car',
        executable='red_box_tracker',
        name='red_box_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Navigation Controller Node
    navigation_controller = Node(
        package='red_box_car',
        executable='navigation_controller',
        name='navigation_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Pickup Controller Node
    pickup_controller = Node(
        package='red_box_car',
        executable='pickup_controller',
        name='pickup_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(car_robot_state_publisher)
    ld.add_action(spawn_car)
    ld.add_action(ros_gz_bridge)
    ld.add_action(camera_simulator)
    ld.add_action(red_box_spawner)
    ld.add_action(red_box_tracker)
    ld.add_action(navigation_controller)
    ld.add_action(pickup_controller)
    
    return ld