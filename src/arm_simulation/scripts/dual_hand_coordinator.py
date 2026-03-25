#!/usr/bin/env python3
"""
Dual Hand Coordinator

Subscribes to:
  - /finger_count (0-10): Splits count across both hands
    left  = min(count, 5)
    right = max(0, count - 5)
    
  - /letter_command: Handles letters and two-character words
    Single letter or word → both hands display it
    Two-character word → right hand shows 1st char, left hand shows 2nd char

Examples:
  finger_count: 8  → left=5, right=3
  finger_count: 10 → left=5, right=5
  letter_command: "a" → both hands display "a"
  letter_command: "ab" → right hand displays "a", left hand displays "b"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class DualHandCoordinator(Node):

    def __init__(self):
        super().__init__('dual_hand_coordinator')

        # Publishers for finger counts
        self.left_finger_pub  = self.create_publisher(Int32, '/left/finger_count',  10)
        self.right_finger_pub = self.create_publisher(Int32, '/right/finger_count', 10)

        # Publishers for letter commands
        self.left_letter_pub  = self.create_publisher(String, '/left/letter_command',  10)
        self.right_letter_pub = self.create_publisher(String, '/right/letter_command', 10)

        # Subscribers
        self.create_subscription(Int32, '/finger_count', self.finger_count_callback, 10)
        self.create_subscription(String, '/letter_command', self.letter_command_callback, 10)

        self.get_logger().info('Dual Hand Coordinator ready')
        self.get_logger().info('Send /finger_count (0-10) to control both hands')
        self.get_logger().info('Send /letter_command for letters/words:')
        self.get_logger().info('  Single letter → both hands display it')
        self.get_logger().info('  Two-char word → right hand displays 1st char, left hand displays 2nd char')

    def finger_count_callback(self, msg):
        """Split finger count across both hands"""
        total = max(0, min(10, msg.data))

        left_count  = min(total, 5)
        right_count = max(0, total - 5)

        left_msg = Int32()
        left_msg.data = left_count
        self.left_finger_pub.publish(left_msg)

        right_msg = Int32()
        right_msg.data = right_count
        self.right_finger_pub.publish(right_msg)

        self.get_logger().info(
            f'Finger count: Total={total} → left={left_count}, right={right_count}')

    def letter_command_callback(self, msg):
        """Handle letter/word commands with dual-hand support for two-character words"""
        command = msg.data.strip()
        
        if not command:
            return
        
        # Handle two-character words: right hand gets 1st char, left hand gets 2nd char
        if len(command) == 2 and command.isalpha():
            self.get_logger().info(
                f'Two-char word: "{command}" → right hand: "{command[0]}", left hand: "{command[1]}"')
            
            # Send first character to right hand
            right_msg = String()
            right_msg.data = command[0]
            self.right_letter_pub.publish(right_msg)
            
            # Send second character to left hand
            left_msg = String()
            left_msg.data = command[1]
            self.left_letter_pub.publish(left_msg)
        
        # Single character or other multi-character text: send to both hands
        else:
            self.get_logger().info(f'Letter command: "{command}" → both hands')
            
            msg_out = String()
            msg_out.data = command
            self.left_letter_pub.publish(msg_out)
            self.right_letter_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = DualHandCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
