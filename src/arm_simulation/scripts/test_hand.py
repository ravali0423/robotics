#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class HandTester(Node):
    def __init__(self):
        super().__init__('hand_tester')
        self.publisher = self.create_publisher(Int32, '/finger_count', 10)
        
    def test_finger_counts(self):
        """Test all finger configurations from 0 to 5"""
        self.get_logger().info("Starting finger count demonstration...")
        
        for count in range(6):  # 0 to 5
            msg = Int32()
            msg.data = count
            self.publisher.publish(msg)
            self.get_logger().info(f"Showing {count} finger(s)")
            time.sleep(3)  # Wait 3 seconds between changes
        
        self.get_logger().info("Demonstration complete!")

def main(args=None):
    rclpy.init(args=args)
    tester = HandTester()
    
    # Wait a moment for the hand controller to be ready
    time.sleep(2)
    
    # Run the test
    tester.test_finger_counts()
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()