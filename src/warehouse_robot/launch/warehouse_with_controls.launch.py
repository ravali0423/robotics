#!/usr/bin/env python3
"""
Warehouse Robot with Movement Controls Launch File

This launch file starts the warehouse simulation with joystick and keyboard control options.
Includes the robot, world, bridges, and control interfaces.
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    TimerAction,
    GroupAction,
    OpaqueFunction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def read_waypoints():
    """Read generated waypoints from file."""
    waypoints_file = "/home/ravali/ros2_ws/src/warehouse_robot/scripts/waypoints.txt"
    waypoints = {}
    
    try:
        with open(waypoints_file, 'r') as f:
            for line in f:
                key, value = line.strip().split('=')
                waypoints[key] = float(value)
        
        print("📍 Waypoints loaded:", waypoints)
        return waypoints
    except Exception as e:
        print(f"Warning: Could not read waypoints file: {e}")
        # Return default values
        return {
            'START_X': 0.0,
            'START_Y': 0.0,
            'PACKAGE_X': 2.0,
            'PACKAGE_Y': 2.0,
            'DEST_X': 5.0,
            'DEST_Y': 5.0
        }


def launch_setup(context, *args, **kwargs):
    """Setup function called when launch arguments are available"""
    
    # Get launch configuration values
    control_mode = LaunchConfiguration('control_mode').perform(context)
    enable_joystick = LaunchConfiguration('enable_joystick').perform(context)
    enable_keyboard = LaunchConfiguration('enable_keyboard').perform(context)
    
    # Read waypoints for spawn position
    waypoints = read_waypoints()
    spawn_x = waypoints.get('START_X', 0.0)
    spawn_y = waypoints.get('START_Y', 0.0)
    spawn_z = LaunchConfiguration('spawn_z').perform(context)
    
    print(f"🚗 Robot spawn position: ({spawn_x:.2f}, {spawn_y:.2f}, {spawn_z})")
    print(f"🎮 Control mode: {control_mode}")
    
    # Package paths
    warehouse_robot_pkg = FindPackageShare('warehouse_robot')
    
    # World file path
    warehouse_world_path = PathJoinSubstitution([
        warehouse_robot_pkg,
        'worlds',
        'warehouse_world.sdf'
    ])
    
    # Start Gazebo with warehouse world
    start_gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            warehouse_world_path,
            '-v', '4'
        ],
        output='screen'
    )
    
    # Start physics simulation
    start_physics = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/warehouse_world/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', 'pause: false'
                ],
                output='screen'
            )
        ]
    )
    
    # Spawn warehouse robot
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/warehouse_world/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', 
                    'sdf_filename: "/home/ravali/ros2_ws/src/warehouse_robot/models/diff_drive/model.sdf", '
                    'name: "warehouse_car", '
                    'pose: {'
                    '  position: {'
                    f'    x: {spawn_x}, '
                    f'    y: {spawn_y}, '
                    f'    z: {spawn_z}'
                    '  }'
                    '}'
                ],
                output='screen'
            )
        ]
    )
    
    # ROS-Gazebo bridge for robot control
    robot_bridge = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/model/warehouse_car/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/warehouse_car/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
                ],
                output='screen',
                name='robot_bridge'
            )
        ]
    )
    
    # Control nodes based on configuration
    control_nodes = []
    
    # Joystick controller (requires websockets)
    if enable_joystick.lower() == 'true':
        joystick_controller = TimerAction(
            period=8.0,  # Start after bridge is ready
            actions=[
                Node(
                    package='warehouse_robot',
                    executable='joystick_controller.py',
                    name='joystick_controller',
                    output='screen'
                )
            ]
        )
        control_nodes.append(joystick_controller)
        print("🕹️ Joystick controller will be started")
        print("📱 Open joystick_controller.html in your browser for web control")
    
    # Keyboard controller
    if enable_keyboard.lower() == 'true':
        keyboard_controller = TimerAction(
            period=8.0,  # Start after bridge is ready
            actions=[
                Node(
                    package='warehouse_robot',
                    executable='keyboard_controller.py',
                    name='keyboard_controller',
                    output='screen'
                )
            ]
        )
        control_nodes.append(keyboard_controller)
        print("⌨️ Keyboard controller will be started")
        print("🎮 Use WASD keys to control the robot")
    
    # Information node to display control instructions
    info_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='warehouse_robot',
                executable='display_control_info.py',
                name='control_info_display',
                output='screen',
                parameters=[
                    {'joystick_enabled': enable_joystick.lower() == 'true'},
                    {'keyboard_enabled': enable_keyboard.lower() == 'true'}
                ]
            )
        ]
    )
    
    # Create all actions
    actions = [
        start_gazebo,
        start_physics,
        spawn_robot,
        robot_bridge,
        info_node
    ] + control_nodes
    
    return actions


def generate_launch_description():
    # Launch arguments
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='X position to spawn robot'
    )
    
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Y position to spawn robot'
    )
    
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='5.0',
        description='Z position to spawn robot (for drop test)'
    )
    
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='both',
        description='Control mode: joystick, keyboard, or both'
    )
    
    enable_joystick_arg = DeclareLaunchArgument(
        'enable_joystick',
        default_value='true',
        description='Enable web-based joystick controller'
    )
    
    enable_keyboard_arg = DeclareLaunchArgument(
        'enable_keyboard',
        default_value='true',
        description='Enable keyboard controller'
    )
    
    # Use OpaqueFunction to access launch arguments in setup
    launch_setup_action = OpaqueFunction(function=launch_setup)
    
    return LaunchDescription([
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        control_mode_arg,
        enable_joystick_arg,
        enable_keyboard_arg,
        launch_setup_action
    ])