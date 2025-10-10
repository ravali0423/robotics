#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
import math
import threading
import time


class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.navigation_status_pub = self.create_publisher(String, '/navigation_status', 10)
        
        # Subscribers - Remove problematic odometry subscription for now
        # self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.navigation_command_sub = self.create_subscription(String, '/navigation_command', self.navigation_command_callback, 10)
        self.target_position_sub = self.create_subscription(Point, '/target_position', self.target_position_callback, 10)
        
        # Internal state - Use simulated position for now
        self.current_pose = None  # Will use simulated starting position
        self.simulated_x = 0.0    # Simulated robot position
        self.simulated_y = 0.0
        self.simulated_yaw = 0.0
        self.target_position = Point()
        self.navigation_active = False
        self.navigation_goal_reached = False
        self.lock = threading.Lock()
        
        # Navigation parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.position_tolerance = 0.3  # meters
        self.angle_tolerance = 0.1  # radians
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Initialize simulated pose after a short delay
        self.init_timer = self.create_timer(1.0, self.initialize_pose)
        
        self.get_logger().info("Navigation Controller started")
    
    def initialize_pose(self):
        """Initialize simulated pose after startup"""
        with self.lock:
            # Assume robot starts at origin
            self.simulated_x = 0.0
            self.simulated_y = 0.0
            self.simulated_yaw = 0.0
        
        self.get_logger().info("Initialized simulated pose at origin")
        self.init_timer.destroy()  # Only run once
    
    def odom_callback(self, msg):
        """Update current pose from odometry - DISABLED for now"""
        # Commented out to avoid the problematic odometry subscription
        pass
    
    def navigation_command_callback(self, msg):
        """Handle navigation commands"""
        command = msg.data.lower().strip()
        
        if command == "start":
            self.start_navigation()
        elif command == "stop":
            self.stop_navigation()
        elif command == "resume":
            self.resume_navigation()
        else:
            self.get_logger().warn(f"Unknown navigation command: {command}")
    
    def target_position_callback(self, msg):
        """Set new target position"""
        with self.lock:
            self.target_position = msg
            self.navigation_goal_reached = False
        
        self.get_logger().info(f"New target position: x={msg.x:.2f}, y={msg.y:.2f}")
    
    def start_navigation(self):
        """Start navigation to target position"""
        with self.lock:
            self.navigation_active = True
            self.navigation_goal_reached = False
        
        status_msg = String()
        status_msg.data = "navigating"
        self.navigation_status_pub.publish(status_msg)
        
        self.get_logger().info("Navigation started")
    
    def stop_navigation(self):
        """Stop navigation and halt the robot"""
        with self.lock:
            self.navigation_active = False
        
        # Send stop command
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        status_msg = String()
        status_msg.data = "stopped"
        self.navigation_status_pub.publish(status_msg)
        
        self.get_logger().info("Navigation stopped")
    
    def resume_navigation(self):
        """Resume navigation to target"""
        with self.lock:
            if not self.navigation_goal_reached:
                self.navigation_active = True
        
        status_msg = String()
        status_msg.data = "navigating"
        self.navigation_status_pub.publish(status_msg)
        
        self.get_logger().info("Navigation resumed")
    
    def control_loop(self):
        """Main control loop for navigation"""
        try:
            with self.lock:
                if not self.navigation_active or self.navigation_goal_reached:
                    return
                
                # Use simulated position instead of odometry
                current_x = self.simulated_x
                current_y = self.simulated_y
                current_yaw = self.simulated_yaw
                target_pos = self.target_position
            
            # Calculate distance to target
            dx = target_pos.x - current_x
            dy = target_pos.y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Check if we've reached the target
            if distance < self.position_tolerance:
                self.reach_goal()
                return
            
            # Calculate angle to target
            target_angle = math.atan2(dy, dx)
            
            # Calculate angle difference
            angle_diff = self.normalize_angle(target_angle - current_yaw)
            
            # Create twist message
            twist = Twist()
            
            # If we need to turn significantly, turn first
            if abs(angle_diff) > self.angle_tolerance:
                # Only turn
                twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                twist.linear.x = 0.0
                
                # Update simulated orientation
                with self.lock:
                    dt = 0.1  # Control loop period
                    self.simulated_yaw += twist.angular.z * dt
                    self.simulated_yaw = self.normalize_angle(self.simulated_yaw)
            else:
                # Move forward with slight course correction
                twist.linear.x = min(self.linear_speed, distance * 2.0)  # Slow down when close
                twist.angular.z = angle_diff * 2.0  # Proportional steering
                
                # Update simulated position
                with self.lock:
                    dt = 0.1  # Control loop period
                    self.simulated_x += twist.linear.x * math.cos(self.simulated_yaw) * dt
                    self.simulated_y += twist.linear.x * math.sin(self.simulated_yaw) * dt
                    self.simulated_yaw += twist.angular.z * dt
                    self.simulated_yaw = self.normalize_angle(self.simulated_yaw)
            
            # Publish the twist command
            self.cmd_vel_pub.publish(twist)
            
            # Log progress occasionally
            if int(time.time() * 2) % 10 == 0:  # Every 5 seconds
                self.get_logger().info(f"Distance to target: {distance:.2f}m, Current: ({current_x:.2f}, {current_y:.2f}), Target: ({target_pos.x:.2f}, {target_pos.y:.2f})")
                
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
            # Stop the robot in case of error
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
    
    def reach_goal(self):
        """Handle reaching the navigation goal"""
        with self.lock:
            self.navigation_active = False
            self.navigation_goal_reached = True
        
        # Stop the robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Publish status
        status_msg = String()
        status_msg.data = "goal_reached"
        self.navigation_status_pub.publish(status_msg)
        
        self.get_logger().info("Navigation goal reached!")
    
    def get_yaw_from_quaternion(self, orientation):
        """Convert quaternion to yaw angle"""
        import math
        
        # Extract quaternion components
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
        # Convert to yaw
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_navigation_status(self):
        """Get current navigation status"""
        with self.lock:
            return {
                'active': self.navigation_active,
                'goal_reached': self.navigation_goal_reached,
                'target_x': self.target_position.x,
                'target_y': self.target_position.y,
                'current_x': self.current_pose.position.x,
                'current_y': self.current_pose.position.y
            }


def main(args=None):
    rclpy.init(args=args)
    
    navigation_controller = NavigationController()
    
    try:
        rclpy.spin(navigation_controller)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()