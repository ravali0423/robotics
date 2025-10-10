#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class LocationCommander(Node):
    def __init__(self):
        super().__init__('location_commander')
        self.publisher = self.create_publisher(String, '/move_to_location', 10)
        
    def send_location_command(self, location):
        msg = String()
        msg.data = location
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent command: {location}')

def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  ros2 run warehouse_robot move_robot <location>")
        print("  ros2 run warehouse_robot move_robot '{\"x\": 1.0, \"y\": 2.0}'")
        print("  ros2 run warehouse_robot move_robot center")
        print("\nAvailable named locations:")
        print("  center, origin, corner1, corner2, corner3, corner4")
        print("  wall1, wall2, front, back, left, right")
        return
    
    location = ' '.join(sys.argv[1:])  # Join all arguments as location command
    
    rclpy.init()
    commander = LocationCommander()
    
    # Wait a moment for publisher to be ready
    import time
    time.sleep(0.5)
    
    commander.send_location_command(location)
    
    # Keep node alive briefly to ensure message is sent
    time.sleep(1.0)
    
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()