#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time

class RVizDebugger(Node):
    def __init__(self):
        super().__init__('rviz_debugger')
        
        # Subscribe to topics
        self.robot_description_sub = self.create_subscription(
            String, '/robot_description', self.robot_description_callback, 10
        )
        
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )
        
        self.robot_description_received = False
        self.joint_states_received = False
        
        # Timer to check status
        self.timer = self.create_timer(2.0, self.check_status)
        
        self.get_logger().info("RViz Debugger started. Checking robot description and joint states...")

    def robot_description_callback(self, msg):
        if not self.robot_description_received:
            self.get_logger().info("✓ Robot description received!")
            self.get_logger().info(f"URDF length: {len(msg.data)} characters")
            self.robot_description_received = True

    def joint_states_callback(self, msg):
        if not self.joint_states_received:
            self.get_logger().info("✓ Joint states received!")
            self.get_logger().info(f"Joint names: {msg.name}")
            self.get_logger().info(f"Joint positions: {msg.position}")
            self.joint_states_received = True

    def check_status(self):
        self.get_logger().info("=== RViz Debug Status ===")
        self.get_logger().info(f"Robot Description: {'✓' if self.robot_description_received else '✗'}")
        self.get_logger().info(f"Joint States: {'✓' if self.joint_states_received else '✗'}")
        
        if self.robot_description_received and self.joint_states_received:
            self.get_logger().info("All topics are working! The issue might be in RViz configuration.")
            self.get_logger().info("Try the following in RViz:")
            self.get_logger().info("1. Click 'Add' -> 'RobotModel'")
            self.get_logger().info("2. Set Robot Description Topic to '/robot_description'")
            self.get_logger().info("3. Set TF Prefix to empty")
            self.get_logger().info("4. Set Fixed Frame to 'world'")
            self.get_logger().info("5. Click 'Reset' in Global Options")

def main():
    rclpy.init()
    debugger = RVizDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        pass
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()