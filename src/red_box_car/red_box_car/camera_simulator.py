#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        
        # Create publisher for camera image
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create timer to publish images at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.publish_camera_image)
        
        # Initialize frame counter for animated content
        self.frame_count = 0
        
        self.get_logger().info('Camera simulator started - Publishing to /camera/image_raw')

    def publish_camera_image(self):
        """Publish a simulated camera image"""
        
        # Create a simulated camera image (640x480)
        height, width = 480, 640
        
        # Create a gradient background that changes over time
        background = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Animated gradient
        for y in range(height):
            for x in range(width):
                r = int(128 + 127 * np.sin(x * 0.01 + self.frame_count * 0.1))
                g = int(128 + 127 * np.sin(y * 0.01 + self.frame_count * 0.05))
                b = int(128 + 127 * np.sin((x + y) * 0.005 + self.frame_count * 0.02))
                background[y, x] = [r, g, b]
        
        # Add some geometric shapes
        # Moving circle
        center_x = int(320 + 200 * np.sin(self.frame_count * 0.05))
        center_y = int(240 + 150 * np.cos(self.frame_count * 0.03))
        cv2.circle(background, (center_x, center_y), 30, (255, 255, 255), -1)
        
        # Add text overlay
        cv2.putText(background, f'Camera Feed - Frame {self.frame_count}', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(background, 'Red Box Car Camera Simulator', 
                   (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add floor line to show ground reference
        floor_y = 400
        cv2.line(background, (0, floor_y), (width, floor_y), (100, 200, 100), 2)
        cv2.putText(background, 'Floor', (10, floor_y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 100), 1)
        
        # Add a red box in the scene for visual interest (smaller, in background)
        box_x = int(450 + 50 * np.cos(self.frame_count * 0.02))
        box_y = int(150 + 30 * np.sin(self.frame_count * 0.04))
        cv2.rectangle(background, (box_x, box_y), (box_x + 60, box_y + 45), (0, 0, 255), -1)
        cv2.putText(background, 'RED BOX', (box_x + 5, box_y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(background, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_link'
        
        # Publish the image
        self.image_publisher.publish(ros_image)
        
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    
    camera_simulator = CameraSimulator()
    
    try:
        rclpy.spin(camera_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        camera_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()