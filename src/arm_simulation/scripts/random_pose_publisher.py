#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random
import math

class UR5RandomPosePublisher(Node):
    def __init__(self):
        super().__init__('ur5_random_pose_publisher')
        
        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # UR5 joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # UR5 joint limits (in radians)
        self.joint_limits = {
            'shoulder_pan_joint': (-6.28, 6.28),
            'shoulder_lift_joint': (-6.28, 6.28),
            'elbow_joint': (-3.14, 3.14),
            'wrist_1_joint': (-6.28, 6.28),
            'wrist_2_joint': (-6.28, 6.28),
            'wrist_3_joint': (-6.28, 6.28)
        }
        
        # Timer for publishing random poses
        self.timer = self.create_timer(2.0, self.publish_random_pose)
        
        self.get_logger().info('UR5 Random Pose Publisher started!')
        self.get_logger().info('Publishing random robot poses every 2 seconds...')
        self.get_logger().info('Press Ctrl+C to stop')

    def publish_random_pose(self):
        """Publish a random joint configuration"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        # Generate random joint positions within limits
        joint_state.position = []
        for joint_name in self.joint_names:
            lower, upper = self.joint_limits[joint_name]
            random_position = random.uniform(lower, upper)
            joint_state.position.append(random_position)
        
        # Publish the joint state
        self.joint_state_publisher.publish(joint_state)
        
        # Log the random pose
        pose_str = ", ".join([f"{name}: {pos:.2f}" for name, pos in zip(self.joint_names, joint_state.position)])
        self.get_logger().info(f'Random pose: {pose_str}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = UR5RandomPosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()