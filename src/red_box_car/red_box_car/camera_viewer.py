#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Create a subscriber to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Create a CV bridge to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()
        
        self.get_logger().info('Camera viewer started. Press "q" to quit.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Display the image
            cv2.imshow('Camera Feed', cv_image)
            
            # Process OpenCV events
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Shutting down camera viewer...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    camera_viewer = CameraViewer()
    
    try:
        rclpy.spin(camera_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        cv2.destroyAllWindows()
        camera_viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()