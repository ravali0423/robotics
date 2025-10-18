import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_SMARTmBOT_gazebo = FindPackageShare('SMARTmBOT_gazebo')
    pkg_SMARTmBOT_description = FindPackageShare('SMARTmBOT_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # Paths
    world_file = PathJoinSubstitution([pkg_SMARTmBOT_gazebo, 'worlds', 'smartmbot_world.sdf'])
    urdf_file = PathJoinSubstitution([pkg_SMARTmBOT_description, 'urdf', 'smartmbot.urdf.xacro'])

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to the Gazebo world file'
    )

    # Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', LaunchConfiguration('world')]
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_file,
            'use_sim_time': True
        }],
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'smartmbot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.13'
        ],
        output='screen',
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])