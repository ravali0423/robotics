#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import math

class HandController(Node):
    def __init__(self):
        super().__init__('hand_controller')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscriber for finger count commands
        self.count_sub = self.create_subscription(
            Int32,
            '/finger_count',
            self.finger_count_callback,
            10
        )
        
        # Timer to continuously publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Joint names for the hand
        self.joint_names = [
            # Thumb joints
            'thumb_base_joint', 'thumb_proximal_joint',
            # Index finger joints
            'index_base_joint', 'index_proximal_joint', 'index_middle_joint',
            # Middle finger joints
            'middle_base_joint', 'middle_proximal_joint', 'middle_middle_joint',
            # Ring finger joints
            'ring_base_joint', 'ring_proximal_joint', 'ring_middle_joint',
            # Pinky finger joints
            'pinky_base_joint', 'pinky_proximal_joint', 'pinky_middle_joint'
        ]
        
        # Current joint positions (all fingers closed initially)
        self.joint_positions = [0.0] * len(self.joint_names)
        
        # Finger configurations for different counts
        self.finger_configs = {
            0: self.get_closed_hand_config(),
            1: self.get_one_finger_config(),
            2: self.get_two_finger_config(),
            3: self.get_three_finger_config(),
            4: self.get_four_finger_config(),
            5: self.get_five_finger_config()
        }
        
        # Current finger count
        self.current_count = 0
        
        self.get_logger().info('Hand Controller node started. Ready to receive finger count commands.')
        self.get_logger().info('Send commands via: ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 1}"')
    
    def get_closed_hand_config(self):
        """All fingers closed (fist)"""
        return {
            'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
            'index_base_joint': 0.0, 'index_proximal_joint': 1.4, 'index_middle_joint': 1.2,
            'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
            'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
            'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
        }
    
    def get_one_finger_config(self):
        """Only index finger extended"""
        config = self.get_closed_hand_config()
        config.update({
            'index_base_joint': 0.0,
            'index_proximal_joint': 0.0,
            'index_middle_joint': 0.0
        })
        return config
    
    def get_two_finger_config(self):
        """Index and middle fingers extended"""
        config = self.get_closed_hand_config()
        config.update({
            'index_base_joint': 0.0,
            'index_proximal_joint': 0.0,
            'index_middle_joint': 0.0,
            'middle_base_joint': 0.0,
            'middle_proximal_joint': 0.0,
            'middle_middle_joint': 0.0
        })
        return config
    
    def get_three_finger_config(self):
        """Index, middle, and ring fingers extended"""
        config = self.get_closed_hand_config()
        config.update({
            'index_base_joint': 0.0,
            'index_proximal_joint': 0.0,
            'index_middle_joint': 0.0,
            'middle_base_joint': 0.0,
            'middle_proximal_joint': 0.0,
            'middle_middle_joint': 0.0,
            'ring_base_joint': 0.0,
            'ring_proximal_joint': 0.0,
            'ring_middle_joint': 0.0
        })
        return config
    
    def get_four_finger_config(self):
        """All fingers except thumb extended"""
        config = self.get_closed_hand_config()
        config.update({
            'index_base_joint': 0.0,
            'index_proximal_joint': 0.0,
            'index_middle_joint': 0.0,
            'middle_base_joint': 0.0,
            'middle_proximal_joint': 0.0,
            'middle_middle_joint': 0.0,
            'ring_base_joint': 0.0,
            'ring_proximal_joint': 0.0,
            'ring_middle_joint': 0.0,
            'pinky_base_joint': 0.0,
            'pinky_proximal_joint': 0.0,
            'pinky_middle_joint': 0.0
        })
        return config
    
    def get_five_finger_config(self):
        """All fingers extended (open hand)"""
        return {
            'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
            'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
            'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
            'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
            'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
        }
    
    def finger_count_callback(self, msg):
        """Callback for finger count commands"""
        count = msg.data
        
        if count < 0 or count > 5:
            self.get_logger().warn(f'Invalid finger count: {count}. Must be between 0 and 5.')
            return
        
        self.current_count = count
        self.get_logger().info(f'Setting hand to show {count} finger(s)')
        
        # Update joint positions based on the count
        if count in self.finger_configs:
            config = self.finger_configs[count]
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in config:
                    self.joint_positions[i] = config[joint_name]
    
    def publish_joint_states(self):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.joint_positions
        joint_state.velocity = [0.0] * len(self.joint_names)
        joint_state.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    
    hand_controller = HandController()
    
    try:
        rclpy.spin(hand_controller)
    except KeyboardInterrupt:
        pass
    finally:
        hand_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()