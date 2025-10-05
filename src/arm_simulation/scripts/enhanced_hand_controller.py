#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseArray, Pose
import math
import time

class EnhancedHandController(Node):
    """
    Enhanced hand controller that supports complex ASL gestures beyond simple finger counting.
    Contains basic ASL alphabet and gesture configurations.
    """
    
    def __init__(self):
        super().__init__('enhanced_hand_controller')
        
        # Initialize basic ASL alphabet configurations
        self.asl_alphabet = self._init_basic_asl_alphabet()
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.gesture_status_pub = self.create_publisher(String, '/current_gesture', 10)
        
        # Subscribers
        self.finger_count_sub = self.create_subscription(
            Int32, '/finger_count', self.finger_count_callback, 10)
        
        self.gesture_command_sub = self.create_subscription(
            String, '/gesture_command', self.gesture_command_callback, 10)
        
        self.letter_command_sub = self.create_subscription(
            String, '/letter_command', self.letter_command_callback, 10)
        
        # Timer for continuous joint state publishing
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Joint configuration
        self.joint_names = [
            'thumb_base_joint', 'thumb_proximal_joint',
            'index_base_joint', 'index_proximal_joint', 'index_middle_joint',
            'middle_base_joint', 'middle_proximal_joint', 'middle_middle_joint',
            'ring_base_joint', 'ring_proximal_joint', 'ring_middle_joint',
            'pinky_base_joint', 'pinky_proximal_joint', 'pinky_middle_joint'
        ]
        
        # Current joint positions (start in neutral position)
        self.current_config = self._get_neutral_position()
        self.joint_positions = self._config_to_positions(self.current_config)
        
        # Target positions for smooth transitions
        self.target_positions = self.joint_positions.copy()
        
        # Transition parameters
        self.transition_speed = 0.05  # How fast to move between positions
        self.is_transitioning = False
        
        # Current gesture info
        self.current_gesture_name = "neutral"
        self.gesture_start_time = time.time()
        
        # Original finger counting configurations for backward compatibility
        self.finger_configs = self._init_finger_counting_configs()
        
        self.get_logger().info('Enhanced Hand Controller initialized')
        self.get_logger().info('Available commands:')
        self.get_logger().info('  - /finger_count (Int32): 0-5 for finger counting')
        self.get_logger().info('  - /gesture_command (String): "hello", "goodbye", etc.')
        self.get_logger().info('  - /letter_command (String): "a", "b", "c", ... for finger spelling')
    
    def _init_basic_asl_alphabet(self):
        """Initialize basic ASL alphabet configurations"""
        return {
            'a': {  # Closed fist with thumb on side
                'thumb_base_joint': -0.5, 'thumb_proximal_joint': 0.3,
                'index_base_joint': 0.0, 'index_proximal_joint': 1.4, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'b': {  # Four fingers extended, thumb across palm
                'thumb_base_joint': 0.8, 'thumb_proximal_joint': 1.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            'c': {  # Curved hand like holding a cup
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.5,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.8, 'index_middle_joint': 0.6,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.8, 'middle_middle_joint': 0.6,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.8, 'ring_middle_joint': 0.6,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.8, 'pinky_middle_joint': 0.6
            },
            'hello': {  # Open hand wave
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            'neutral': {  # Neutral position
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            }
        }
    
    def _get_neutral_position(self):
        """Return neutral hand position"""
        return self.asl_alphabet['neutral']
    
    def _init_finger_counting_configs(self):
        """Initialize finger counting configurations for backward compatibility"""
        return {
            0: self._get_closed_hand_config(),
            1: self._get_one_finger_config(),
            2: self._get_two_finger_config(),
            3: self._get_three_finger_config(),
            4: self._get_four_finger_config(),
            5: self._get_five_finger_config()
        }
    
    def _get_closed_hand_config(self):
        """All fingers closed (fist)"""
        return {
            'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
            'index_base_joint': 0.0, 'index_proximal_joint': 1.4, 'index_middle_joint': 1.2,
            'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
            'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
            'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
        }
    
    def _get_one_finger_config(self):
        """Only index finger extended"""
        config = self._get_closed_hand_config()
        config.update({
            'index_base_joint': 0.0,
            'index_proximal_joint': 0.0,
            'index_middle_joint': 0.0
        })
        return config
    
    def _get_two_finger_config(self):
        """Index and middle fingers extended"""
        config = self._get_closed_hand_config()
        config.update({
            'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
            'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0
        })
        return config
    
    def _get_three_finger_config(self):
        """Index, middle, and ring fingers extended"""
        config = self._get_closed_hand_config()
        config.update({
            'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
            'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
            'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0
        })
        return config
    
    def _get_four_finger_config(self):
        """All fingers except thumb extended"""
        config = self._get_closed_hand_config()
        config.update({
            'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
            'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
            'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
            'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
        })
        return config
    
    def _get_five_finger_config(self):
        """All fingers extended (open hand)"""
        return {
            'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
            'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
            'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
            'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
            'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
        }
    
    def finger_count_callback(self, msg):
        """Handle finger counting commands (backward compatibility)"""
        count = msg.data
        
        if count < 0 or count > 5:
            self.get_logger().warn(f'Invalid finger count: {count}. Must be between 0 and 5.')
            return
        
        if count in self.finger_configs:
            config = self.finger_configs[count]
            self.set_hand_configuration(config, f"finger_count_{count}")
            self.get_logger().info(f'Set hand to show {count} finger(s)')
    
    def gesture_command_callback(self, msg):
        """Handle named gesture commands (words/phrases)"""
        gesture_name = msg.data.lower().strip()
        
        if gesture_name in self.asl_alphabet:
            config = self.asl_alphabet[gesture_name]
            self.set_hand_configuration(config, gesture_name)
            self.get_logger().info(f'Performing gesture: "{gesture_name}"')
        else:
            self.get_logger().warn(f'Unknown gesture: "{gesture_name}"')
            self.get_logger().info(f'Available gestures: {", ".join(self.asl_alphabet.keys())}')
    
    def letter_command_callback(self, msg):
        """Handle single letter finger spelling commands"""
        letter = msg.data.lower().strip()
        
        if len(letter) != 1 or not letter.isalpha():
            self.get_logger().warn(f'Invalid letter: "{letter}". Must be a single alphabetic character.')
            return
        
        if letter in self.asl_alphabet:
            config = self.asl_alphabet[letter]
            self.set_hand_configuration(config, f"letter_{letter}")
            self.get_logger().info(f'Finger spelling letter: "{letter.upper()}"')
        else:
            self.get_logger().warn(f'No configuration found for letter: "{letter}"')
            self.get_logger().info(f'Available letters: {", ".join([k for k in self.asl_alphabet.keys() if len(k) == 1])}')
    
    def set_hand_configuration(self, config: dict, gesture_name: str):
        """
        Set target hand configuration with smooth transition
        
        Args:
            config: Dictionary of joint_name: position mappings
            gesture_name: Name of the gesture for status reporting
        """
        # Update target positions
        self.target_positions = self._config_to_positions(config)
        self.current_config = config
        self.current_gesture_name = gesture_name
        self.gesture_start_time = time.time()
        self.is_transitioning = True
        
        # Publish gesture status
        self.publish_gesture_status(gesture_name)
    
    def _config_to_positions(self, config: dict) -> list:
        """Convert configuration dictionary to position list matching joint_names order"""
        positions = []
        for joint_name in self.joint_names:
            positions.append(config.get(joint_name, 0.0))
        return positions
    
    def _smooth_transition(self):
        """Perform smooth transition between current and target positions"""
        if not self.is_transitioning:
            return
        
        all_reached = True
        for i in range(len(self.joint_positions)):
            diff = self.target_positions[i] - self.joint_positions[i]
            
            if abs(diff) > 0.01:  # Threshold for "reached"
                all_reached = False
                # Move towards target
                step = diff * self.transition_speed
                self.joint_positions[i] += step
            else:
                # Snap to target if very close
                self.joint_positions[i] = self.target_positions[i]
        
        if all_reached:
            self.is_transitioning = False
    
    def publish_joint_states(self):
        """Publish current joint states with smooth transitions"""
        # Update positions with smooth transition
        self._smooth_transition()
        
        # Create and publish joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.joint_positions
        joint_state.velocity = [0.0] * len(self.joint_names)
        joint_state.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(joint_state)
    
    def publish_gesture_status(self, gesture_name: str):
        """Publish current gesture status"""
        msg = String()
        msg.data = gesture_name
        self.gesture_status_pub.publish(msg)
    
    def get_gesture_info(self):
        """Get information about the current gesture"""
        duration = time.time() - self.gesture_start_time
        return {
            'name': self.current_gesture_name,
            'duration': duration,
            'is_transitioning': self.is_transitioning,
            'config': self.current_config
        }
    
    def reset_to_neutral(self):
        """Reset hand to neutral position"""
        neutral_config = self._get_neutral_position()
        self.set_hand_configuration(neutral_config, "neutral")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        hand_controller = EnhancedHandController()
        rclpy.spin(hand_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'hand_controller' in locals():
            hand_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()