import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch the complete Speech-to-Gesture system for deaf assistance.
    This system translates spoken words into sign language gestures using a robotic hand.
    """
    
    # Get the URDF file path for the robotic hand
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
        'speech_to_gesture.rviz'
    )

    nodes_to_launch = [
        # Robot State Publisher - publishes the robot's state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),
        
        # Enhanced Hand Controller - controls the robotic hand gestures
        Node(
            package='arm_simulation',
            executable='enhanced_hand_controller.py',
            name='enhanced_hand_controller',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Speech Recognition Node - converts speech to text
        Node(
            package='arm_simulation',
            executable='speech_recognition_node.py',
            name='speech_recognition_node',
            output='screen',
            parameters=[{
                'energy_threshold': 300,
                'pause_threshold': 0.8,
                'language': 'en-US',
                'auto_start': True
            }]
        ),
        
        # Gesture Sequencer - manages gesture sequences
        Node(
            package='arm_simulation',
            executable='gesture_sequencer.py',
            name='gesture_sequencer',
            output='screen',
            parameters=[{
                'letter_duration': 1.5,
                'word_pause': 2.0,
                'letter_pause': 0.8,
                'gesture_duration': 2.5
            }]
        ),
        
        # Speech-to-Gesture Coordinator - main orchestrator
        Node(
            package='arm_simulation',
            executable='speech_to_gesture_coordinator.py',
            name='speech_to_gesture_coordinator',
            output='screen',
            parameters=[{
                'auto_start_listening': True,
                'gesture_timeout': 30.0,
                'min_text_length': 2,
                'max_text_length': 200
            }]
        ),
        
        # Audio-Visual Feedback - provides user feedback
        Node(
            package='arm_simulation',
            executable='audio_visual_feedback.py',
            name='audio_visual_feedback',
            output='screen',
            parameters=[{
                'audio_enabled': True,
                'visual_enabled': True,
                'use_system_sounds': True,
                'use_text_to_speech': True
            }]
        )
    ]

    # Add RViz with config if it exists, otherwise without config
    if os.path.exists(rviz_config_file):
        nodes_to_launch.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file]
            )
        )
    else:
        nodes_to_launch.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen'
            )
        )

    return LaunchDescription(nodes_to_launch)