#!/usr/bin/env python3
"""
Simple Robot Control Test

This script demonstrates how to control the warehouse robot with different movement patterns.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/model/warehouse_car/cmd_vel', 
            10
        )
        
        self.get_logger().info('🎮 Simple Robot Controller started!')
        
    def move_robot(self, linear_x=0.0, angular_z=0.0, duration=2.0):
        """Move robot with specified velocities for a duration"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        
        self.get_logger().info(f'🚗 Moving: linear={linear_x:.2f} m/s, angular={angular_z:.2f} rad/s for {duration}s')
        
        # Publish the command repeatedly for the duration
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
        
        # Stop the robot
        stop_twist = Twist()
        self.cmd_vel_publisher.publish(stop_twist)
        self.get_logger().info('🛑 Stopped')
        
    def demo_movements(self):
        """Demonstrate various robot movements"""
        self.get_logger().info('🚀 Starting movement demonstration...')
        
        time.sleep(2)  # Wait for setup
        
        # Move forward
        self.move_robot(linear_x=0.3, duration=3.0)
        time.sleep(1)
        
        # Move backward
        self.move_robot(linear_x=-0.3, duration=2.0)
        time.sleep(1)
        
        # Turn left
        self.move_robot(angular_z=0.5, duration=2.0)
        time.sleep(1)
        
        # Turn right
        self.move_robot(angular_z=-0.5, duration=2.0)
        time.sleep(1)
        
        # Move forward while turning left (circular motion)
        self.move_robot(linear_x=0.2, angular_z=0.3, duration=4.0)
        time.sleep(1)
        
        self.get_logger().info('✅ Movement demonstration completed!')

def main():
    rclpy.init()
    
    controller = SimpleRobotController()
    
    try:
        controller.demo_movements()
        
        # Keep node alive for manual control if needed
        controller.get_logger().info('Node ready for manual control...')
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Stopping controller...')
    finally:
        # Ensure robot stops
        stop_twist = Twist()
        controller.cmd_vel_publisher.publish(stop_twist)
        rclpy.shutdown()

if __name__ == '__main__':
    main()