#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import json

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.location_sub = self.create_subscription(String, '/move_to_location', self.location_callback, 10)
        
        # Robot state
        self.current_pose = Point()
        self.current_yaw = 0.0
        self.target_pose = None
        self.target_yaw = None
        self.is_moving = False
        
        # Control parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.8  # rad/s
        self.position_tolerance = 0.1  # meters
        self.angle_tolerance = 0.1  # radians
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Robot Controller initialized. Send location commands to /move_to_location topic.')
        self.get_logger().info('Format: {"x": 1.0, "y": 2.0} or "location_name"')
        
    def odom_callback(self, msg):
        """Update robot's current position from odometry"""
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw angle
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def location_callback(self, msg):
        """Handle location command messages"""
        command = msg.data.strip()
        self.get_logger().info(f'Received location command: {command}')
        
        try:
            # Try to parse as JSON coordinates
            if command.startswith('{') and command.endswith('}'):
                coords = json.loads(command)
                target_x = float(coords['x'])
                target_y = float(coords['y'])
                self.move_to_position(target_x, target_y)
            else:
                # Handle predefined location names
                self.move_to_named_location(command.lower())
                
        except Exception as e:
            self.get_logger().error(f'Failed to parse location command: {e}')
            self.get_logger().info('Use format: {"x": 1.0, "y": 2.0} or predefined names like "center", "corner1", etc.')
    
    def move_to_named_location(self, location_name):
        """Move to predefined locations"""
        locations = {
            'center': (0.0, 0.0),
            'origin': (0.0, 0.0),
            'corner1': (3.0, 3.0),
            'corner2': (-3.0, 3.0),
            'corner3': (-3.0, -3.0),
            'corner4': (3.0, -3.0),
            'wall1': (8.0, 0.0),
            'wall2': (-8.0, 0.0),
            'front': (0.0, 5.0),
            'back': (0.0, -5.0),
            'left': (-5.0, 0.0),
            'right': (5.0, 0.0)
        }
        
        if location_name in locations:
            x, y = locations[location_name]
            self.move_to_position(x, y)
            self.get_logger().info(f'Moving to {location_name}: ({x}, {y})')
        else:
            self.get_logger().warn(f'Unknown location: {location_name}')
            self.get_logger().info(f'Available locations: {list(locations.keys())}')
    
    def move_to_position(self, target_x, target_y):
        """Set target position and start movement"""
        self.target_pose = Point()
        self.target_pose.x = target_x
        self.target_pose.y = target_y
        self.is_moving = True
        
        self.get_logger().info(f'Target set: ({target_x:.2f}, {target_y:.2f})')
        
    def control_loop(self):
        """Main control loop for robot movement"""
        if not self.is_moving or self.target_pose is None:
            return
            
        # Calculate distance and angle to target
        dx = self.target_pose.x - self.current_pose.x
        dy = self.target_pose.y - self.current_pose.y
        distance = math.sqrt(dx*dx + dy*dy)
        target_yaw = math.atan2(dy, dx)
        
        # Check if we've reached the target
        if distance < self.position_tolerance:
            self.stop_robot()
            self.get_logger().info(f'Reached target: ({self.target_pose.x:.2f}, {self.target_pose.y:.2f})')
            self.is_moving = False
            return
        
        # Calculate angle difference
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        
        # Create velocity command
        cmd = Twist()
        
        # If we need to turn significantly, turn first
        if abs(angle_diff) > self.angle_tolerance:
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            cmd.linear.x = 0.1  # Move slowly while turning
        else:
            # Move forward toward target
            cmd.linear.x = min(self.linear_speed, distance * 2.0)  # Slow down as we approach
            cmd.angular.z = angle_diff * 2.0  # Small corrections
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)
        
        # Log progress occasionally
        if self.get_clock().now().nanoseconds % 1000000000 < 100000000:  # Every ~1 second
            self.get_logger().info(f'Moving to target: distance={distance:.2f}m, angle_diff={angle_diff:.2f}rad')
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()