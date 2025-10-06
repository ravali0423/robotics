#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MovementTester(Node):
    def __init__(self):
        super().__init__('movement_tester')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Movement Tester Started')
        
        # Wait for connections
        time.sleep(2)
        
        # Test different movements
        self.test_movements()
    
    def publish_twist(self, linear_x, angular_z, duration=2.0):
        """Publish a twist command for a specific duration"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        self.get_logger().info(f'Testing: linear={linear_x:.2f}, angular={angular_z:.2f} for {duration}s')
        
        # Publish command for duration
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Wait between tests
        time.sleep(1)
    
    def test_movements(self):
        """Test all movement types"""
        self.get_logger().info('Starting movement tests...')
        
        # Test forward (w)
        self.publish_twist(2.0, 0.0, 2.0)
        
        # Test backward (s) 
        self.publish_twist(-2.0, 0.0, 2.0)
        
        # Test turn left (a)
        self.publish_twist(0.0, 2.0, 2.0)
        
        # Test turn right (d)
        self.publish_twist(0.0, -2.0, 2.0)
        
        # Test forward + left (q)
        self.publish_twist(2.0, 2.0, 2.0)
        
        # Test forward + right (e)
        self.publish_twist(2.0, -2.0, 2.0)
        
        self.get_logger().info('Movement tests completed!')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = MovementTester()
        rclpy.spin_once(tester)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()