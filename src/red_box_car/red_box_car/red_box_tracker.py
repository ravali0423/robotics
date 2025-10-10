#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import time


class RedBoxTracker(Node):
    def __init__(self):
        super().__init__('red_box_tracker')
        
        # Publishers
        self.red_box_position_pub = self.create_publisher(Point, '/red_box_position', 10)
        self.red_box_detected_pub = self.create_publisher(Bool, '/red_box_detected', 10)
        
        # For simplification, we'll use a simulated red box position
        # In a real scenario, this would come from camera/vision processing
        self.red_box_position = Point()
        self.red_box_detected = True  # Assume red box is always detected for demo
        
        # Timer to periodically publish red box position
        self.timer = self.create_timer(0.5, self.publish_red_box_status)
        
        # Set a default red box position (this would come from vision system)
        self.red_box_position.x = -4.5  # Approximate spawned position
        self.red_box_position.y = 0.0   # Approximate spawned position  
        self.red_box_position.z = 0.15  # Height of red box
        
        self.get_logger().info("Red Box Tracker started - Using simulated detection")
        self.get_logger().info(f"Simulated red box position: x={self.red_box_position.x}, y={self.red_box_position.y}")
    
    def publish_red_box_status(self):
        """Publish red box position and detection status"""
        try:
            # Publish the position
            self.red_box_position_pub.publish(self.red_box_position)
            
            # Publish detection status
            detected_msg = Bool()
            detected_msg.data = self.red_box_detected
            self.red_box_detected_pub.publish(detected_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing red box status: {str(e)}")
    
    def update_red_box_position(self, x, y, z=0.15):
        """Update the red box position (for testing purposes)"""
        self.red_box_position.x = x
        self.red_box_position.y = y
        self.red_box_position.z = z
        self.get_logger().info(f"Updated red box position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
    
    def set_detection_status(self, detected):
        """Set whether the red box is detected"""
        self.red_box_detected = detected
        if detected:
            self.get_logger().info("Red box detection enabled")
        else:
            self.get_logger().info("Red box detection disabled")
    
    def get_red_box_position(self):
        """Public method to get current red box position"""
        return self.red_box_position, self.red_box_detected


def main(args=None):
    rclpy.init(args=args)
    
    red_box_tracker = RedBoxTracker()
    
    try:
        rclpy.spin(red_box_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        red_box_tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()