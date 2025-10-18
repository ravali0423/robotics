#!/usr/bin/env python3
"""
Simple Robot Movement Test

Tests basic robot movement by publishing velocity commands.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class RobotMovementTest(Node):
    def __init__(self):
        super().__init__('robot_movement_test')
        
        # Create publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/model/warehouse_car/cmd_vel', 
            10
        )
        
        self.get_logger().info('🧪 Robot Movement Test Node started')
        self.run_movement_test()
        
    def publish_velocity(self, linear_x=0.0, angular_z=0.0, duration=2.0):
        """Publish velocity for a specific duration"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        
        self.get_logger().info(f'🚗 Moving: linear={linear_x}, angular={angular_z} for {duration}s')
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
        
        # Stop
        stop_twist = Twist()
        self.cmd_vel_publisher.publish(stop_twist)
        self.get_logger().info('🛑 Stopped')
        
    def run_movement_test(self):
        """Run a series of movement tests"""
        time.sleep(2)  # Wait for setup
        
        self.get_logger().info('🚀 Starting movement test sequence...')
        
        # Test forward movement
        self.publish_velocity(linear_x=0.3, duration=3.0)
        time.sleep(1)
        
        # Test backward movement
        self.publish_velocity(linear_x=-0.3, duration=2.0)
        time.sleep(1)
        
        # Test left turn
        self.publish_velocity(angular_z=0.5, duration=2.0)
        time.sleep(1)
        
        # Test right turn
        self.publish_velocity(angular_z=-0.5, duration=2.0)
        time.sleep(1)
        
        # Test combined movement
        self.publish_velocity(linear_x=0.2, angular_z=0.3, duration=3.0)
        time.sleep(1)
        
        self.get_logger().info('✅ Movement test completed!')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = RobotMovementTest()
        rclpy.spin_once(test_node, timeout_sec=20.0)
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()