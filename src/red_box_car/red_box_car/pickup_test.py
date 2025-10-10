#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import time


class PickupCommandPublisher(Node):
    def __init__(self):
        super().__init__('pickup_command_publisher')
        
        # Publishers
        self.pickup_command_pub = self.create_publisher(String, '/pickup_command', 10)
        
        # Subscribers to monitor status
        self.pickup_status_sub = self.create_subscription(String, '/pickup_status', self.pickup_status_callback, 10)
        self.navigation_status_sub = self.create_subscription(String, '/navigation_status', self.navigation_status_callback, 10)
        self.red_box_position_sub = self.create_subscription(Point, '/red_box_position', self.red_box_position_callback, 10)
        
        self.get_logger().info("Pickup Command Publisher started")
        self.get_logger().info("To start pickup, run: ros2 topic pub /pickup_command std_msgs/String \"data: pickup\" --once")
        
        # Timer to show status
        self.status_timer = self.create_timer(5.0, self.show_status)
        
        self.last_pickup_status = "unknown"
        self.last_navigation_status = "unknown"
        self.last_red_box_position = None
    
    def pickup_status_callback(self, msg):
        self.last_pickup_status = msg.data
        self.get_logger().info(f"Pickup Status: {msg.data}")
    
    def navigation_status_callback(self, msg):
        self.last_navigation_status = msg.data
        self.get_logger().info(f"Navigation Status: {msg.data}")
    
    def red_box_position_callback(self, msg):
        self.last_red_box_position = msg
        self.get_logger().debug(f"Red box at: x={msg.x:.2f}, y={msg.y:.2f}")
    
    def show_status(self):
        """Show current system status"""
        self.get_logger().info("=== System Status ===")
        self.get_logger().info(f"Pickup: {self.last_pickup_status}")
        self.get_logger().info(f"Navigation: {self.last_navigation_status}")
        if self.last_red_box_position:
            self.get_logger().info(f"Red box position: x={self.last_red_box_position.x:.2f}, y={self.last_red_box_position.y:.2f}")
        else:
            self.get_logger().info("Red box position: unknown")
        self.get_logger().info("===================")
    
    def send_pickup_command(self):
        """Send pickup command"""
        msg = String()
        msg.data = "pickup"
        self.pickup_command_pub.publish(msg)
        self.get_logger().info("Pickup command sent!")


def main(args=None):
    rclpy.init(args=args)
    
    pickup_command_publisher = PickupCommandPublisher()
    
    try:
        rclpy.spin(pickup_command_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        pickup_command_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()