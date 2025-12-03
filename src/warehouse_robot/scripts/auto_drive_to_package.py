#!/usr/bin/env python3
"""
Auto Drive to Package

Automatically drives the robot toward the package for collision testing.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math


class AutoDriveToPackage(Node):
    def __init__(self):
        super().__init__('auto_drive_to_package')
        
        # Create publisher for robot velocity
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/model/warehouse_car/cmd_vel', 
            10
        )
        
        # Package location from warehouse world
        self.target_x = -2.9
        self.target_y = -7.2
        
        # Robot start position  
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        self.get_logger().info("🚗 Auto Drive to Package initialized")
        self.get_logger().info(f"📦 Target: ({self.target_x}, {self.target_y})")
        
        # Start driving after a delay
        self.timer = self.create_timer(0.5, self.drive_to_package)
        self.step_count = 0
        
    def drive_to_package(self):
        """Drive robot toward package location."""
        
        # Estimate robot position based on movement (simplified)
        self.step_count += 1
        
        # Calculate direction to package
        dx = self.target_x - self.robot_x
        dy = self.target_y - self.robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        self.get_logger().info(f"📍 Step {self.step_count}: Distance to package: {distance:.2f}m")
        
        if distance > 0.5:  # Keep driving until close
            msg = Twist()
            
            # Simple proportional control
            msg.linear.x = 0.3  # Forward speed
            
            # Calculate needed turn angle
            target_angle = math.atan2(dy, dx)
            # For simplicity, just turn left initially to head toward negative X,Y
            if self.step_count < 10:
                msg.angular.z = -0.3  # Turn left
            elif self.step_count < 25:
                msg.angular.z = 0.0   # Go straight
                msg.linear.x = 0.4
            else:
                msg.angular.z = -0.2  # Fine tune turn
                
            self.cmd_vel_publisher.publish(msg)
            
            # Update estimated position (rough approximation)
            self.robot_x += 0.3 * 0.5 * math.cos(target_angle)
            self.robot_y += 0.3 * 0.5 * math.sin(target_angle)
            
        else:
            # Stop when close
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(msg)
            
            self.get_logger().info("🎯 Reached package location! Stopping...")
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    
    auto_driver = AutoDriveToPackage()
    
    try:
        auto_driver.get_logger().info("🚀 Auto driving to package...")
        rclpy.spin(auto_driver)
    except KeyboardInterrupt:
        auto_driver.get_logger().info("🛑 Auto drive stopped")
    finally:
        # Stop robot
        msg = Twist()
        auto_driver.cmd_vel_publisher.publish(msg)
        
        auto_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()