#!/usr/bin/env python3
"""
Package Collision Handler

This script detects collisions between the robot and packages using real-time
odometry data, and automatically attaches packages to the top of the robot 
chassis when contact is made.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
import subprocess
import time
import threading
import os


class PackageCollisionHandler(Node):
    def __init__(self):
        super().__init__('package_collision_handler')
        
        # Track package attachment state
        self.package_attached = False
        self.attached_package_name = None
        
        # Robot and package info
        self.robot_name = "warehouse_car"
        self.package_name = "pickup_package"
        self.attachment_offset = [0.0, 0.0, 0.7]  # Position on top of robot chassis
        
        # Load waypoints to get actual starting position
        waypoints = self.load_waypoints()
        start_x = waypoints.get('START_X', 5.281)
        start_y = waypoints.get('START_Y', -5.018)
        
        # Robot position from odometry (real-time) - initialize with actual spawn position
        self.robot_position = [start_x, start_y, 0.0]  # Actual spawn position from waypoints
        self.package_position = [waypoints.get('PACKAGE_X', -2.909), waypoints.get('PACKAGE_Y', -7.217), 0.5]  # Package position from waypoints
        
        # Subscribe to robot odometry for real-time position
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/model/warehouse_car/odometry',
            self.odom_callback,
            10
        )
        
        # Contact monitoring thread
        self.contact_thread = threading.Thread(target=self.monitor_contacts, daemon=True)
        self.contact_thread.start()
        
        self.get_logger().info("🔗 Package collision handler initialized")
        self.get_logger().info(f"📦 Monitoring collisions between {self.robot_name} and {self.package_name}")
        self.get_logger().info("🔄 Subscribing to robot odometry for real-time position")
    
    def odom_callback(self, msg):
        """Update robot position from odometry data."""
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y  
        self.robot_position[2] = msg.pose.pose.position.z
    
    def load_waypoints(self):
        """Load waypoints from the waypoints.txt file."""
        waypoints = {}
        try:
            # Try to find waypoints file in the scripts directory
            script_dir = os.path.dirname(os.path.abspath(__file__))
            waypoints_file = os.path.join(script_dir, 'waypoints.txt')
            
            if os.path.exists(waypoints_file):
                with open(waypoints_file, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line and '=' in line:
                            key, value = line.split('=', 1)
                            waypoints[key.strip()] = float(value.strip())
                self.get_logger().info(f"📍 Loaded waypoints: {waypoints}")
            else:
                self.get_logger().warning(f"⚠️ Waypoints file not found at {waypoints_file}, using defaults")
        except Exception as e:
            self.get_logger().error(f"❌ Error loading waypoints: {e}")
        
        return waypoints
    
    def monitor_contacts(self):
        """Monitor contact between robot and package using real-time positions."""
        while rclpy.ok():
            try:
                if not self.package_attached:
                    # Check if robot and package are in contact
                    if self.check_robot_package_contact():
                        self.get_logger().info("💥 Collision detected! Attaching package...")
                        self.attach_package_to_robot()
                
                time.sleep(1.0)  # Check contacts once per second for debugging
                
            except Exception as e:
                self.get_logger().error(f"Contact monitoring error: {e}")
                time.sleep(1.0)
    
    def check_robot_package_contact(self):
        """Check if robot chassis is touching the package."""
        try:
            # Use real-time robot position from odometry and fixed package position
            robot_pose = self.robot_position
            package_pose = self.package_position
            
            # Calculate 3D distance between robot and package centers
            dx = robot_pose[0] - package_pose[0]
            dy = robot_pose[1] - package_pose[1]
            dz = robot_pose[2] - package_pose[2]
            
            # Calculate both 2D (XY) and 3D distances for debugging
            distance_2d = (dx*dx + dy*dy) ** 0.5
            distance_3d = (dx*dx + dy*dy + dz*dz) ** 0.5
            
            # Log current positions and distances for debugging
            self.get_logger().info(f"🤖 Robot: ({robot_pose[0]:.2f}, {robot_pose[1]:.2f}, {robot_pose[2]:.2f})")
            self.get_logger().info(f"📦 Package: ({package_pose[0]:.2f}, {package_pose[1]:.2f}, {package_pose[2]:.2f})")
            self.get_logger().info(f"📏 2D Distance: {distance_2d:.2f}m, 3D Distance: {distance_3d:.2f}m")
            
            # Robot chassis dimensions: 2.01142 x 1 x 0.568726 (length x width x height)
            # Package dimensions: 0.3 x 0.3 x 0.3
            # Robot chassis height is ~0.57m, so robot center is at ~0.285m above ground
            # Package center is at 0.5m height (package bottom + 0.15m to center)
            
            # For collision, check if the robot chassis can reach the package
            robot_half_length = 2.01142 / 2     # ~1.01m (front/back reach)
            robot_half_width = 1.0 / 2          # 0.5m (side reach) 
            robot_height = 0.568726              # Full height of chassis
            package_half_size = 0.3 / 2         # 0.15m (package radius)
            
            # Contact threshold for XY plane (robot can reach package laterally)
            xy_threshold = max(robot_half_length, robot_half_width) + package_half_size + 0.1  # ~1.16m
            
            # Z-axis collision check: robot top should be able to reach package bottom
            robot_top_z = robot_pose[2] + robot_height  # Robot top surface
            package_bottom_z = package_pose[2] - package_half_size  # Package bottom
            
            # Check if robot is close enough in XY and Z ranges overlap
            xy_collision = distance_2d < xy_threshold
            z_collision = robot_top_z >= package_bottom_z and robot_pose[2] <= (package_pose[2] + package_half_size)
            
            if xy_collision and z_collision:
                self.get_logger().info(f"💥 COLLISION DETECTED!")
                self.get_logger().info(f"   XY Distance: {distance_2d:.2f}m < {xy_threshold:.2f}m ✓")
                self.get_logger().info(f"   Robot top: {robot_top_z:.2f}m >= Package bottom: {package_bottom_z:.2f}m ✓")
                return True
            else:
                collision_status = []
                if not xy_collision:
                    collision_status.append(f"XY: {distance_2d:.2f}m > {xy_threshold:.2f}m")
                if not z_collision:
                    collision_status.append(f"Z: robot_top={robot_top_z:.2f}m, pkg_bottom={package_bottom_z:.2f}m")
                self.get_logger().debug(f"No collision - {', '.join(collision_status)}")
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"Contact check error: {e}")
            return False
    
    def attach_package_to_robot(self):
        """Attach the package to the top of the robot chassis."""
        try:
            if self.package_attached:
                self.get_logger().warning("📦 Package already attached!")
                return
            
            # Get current robot pose
            robot_pose = self.robot_position
            
            # Calculate attachment position (on top of robot)
            attach_x = robot_pose[0] + self.attachment_offset[0]
            attach_y = robot_pose[1] + self.attachment_offset[1]
            attach_z = robot_pose[2] + self.attachment_offset[2]
            
            # Move package to attachment position
            self.get_logger().info(f"📍 Moving package to attachment position: ({attach_x:.2f}, {attach_y:.2f}, {attach_z:.2f})")
            
            move_cmd = [
                'gz', 'service', '-s', f'/world/warehouse_world/set_entity_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', 
                f'name: "{self.package_name}", '
                f'position: {{x: {attach_x}, y: {attach_y}, z: {attach_z}}}, '
                f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
            ]
            
            move_result = subprocess.run(move_cmd, capture_output=True, text=True, timeout=5.0)
            
            if move_result.returncode != 0:
                self.get_logger().error(f"❌ Failed to move package: {move_result.stderr}")
                return
            
            # Create fixed joint to attach package to robot
            self.get_logger().info("🔗 Creating attachment joint...")
            
            joint_cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/create_joint',
                '--reqtype', 'gz.msgs.Joint',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req',
                f'name: "package_attachment_joint", '
                f'type: FIXED, '
                f'parent: "{self.robot_name}::chassis", '
                f'child: "{self.package_name}::package_link", '
                f'pose: {{position: {{x: {self.attachment_offset[0]}, y: {self.attachment_offset[1]}, z: {self.attachment_offset[2]}}}, '
                f'orientation: {{x: 0, y: 0, z: 0, w: 1}}}}'
            ]
            
            joint_result = subprocess.run(joint_cmd, capture_output=True, text=True, timeout=5.0)
            
            if joint_result.returncode == 0:
                self.package_attached = True
                self.attached_package_name = self.package_name
                self.get_logger().info("✅ Package successfully attached to robot!")
                self.get_logger().info("📦 Package will now move with the robot")
            else:
                self.get_logger().error(f"❌ Failed to create joint: {joint_result.stderr}")
                # Try alternative approach - continuously update package position
                self.start_package_following()
        
        except Exception as e:
            self.get_logger().error(f"Attachment error: {e}")
            # Fallback to position following
            self.start_package_following()
    
    def start_package_following(self):
        """Alternative method: continuously update package position to follow robot."""
        self.package_attached = True
        self.attached_package_name = self.package_name
        self.get_logger().info("🔄 Starting position-based package following")
        
        # Start following thread
        follow_thread = threading.Thread(target=self.follow_robot, daemon=True)
        follow_thread.start()
    
    def follow_robot(self):
        """Continuously update package position to follow robot."""
        while rclpy.ok() and self.package_attached:
            try:
                robot_pose = self.robot_position
                
                # Calculate new package position
                new_x = robot_pose[0] + self.attachment_offset[0]
                new_y = robot_pose[1] + self.attachment_offset[1]
                new_z = robot_pose[2] + self.attachment_offset[2]
                
                # Update package position
                cmd = [
                    'gz', 'service', '-s', f'/world/warehouse_world/set_entity_pose',
                    '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', 
                    f'name: "{self.package_name}", '
                    f'position: {{x: {new_x}, y: {new_y}, z: {new_z}}}, '
                    f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
                ]
                
                subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
                
                time.sleep(0.05)  # Update 20 times per second
                
            except Exception as e:
                self.get_logger().debug(f"Following error: {e}")
                time.sleep(0.1)
    
    def detach_package(self):
        """Detach the package from the robot."""
        if not self.package_attached:
            self.get_logger().warning("📦 No package attached")
            return
        
        try:
            # Remove joint if it exists
            detach_cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/remove_joint',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', f'name: "package_attachment_joint"'
            ]
            
            subprocess.run(detach_cmd, capture_output=True, text=True, timeout=3.0)
            
            self.package_attached = False
            self.attached_package_name = None
            
            self.get_logger().info("🔓 Package detached from robot")
            
        except Exception as e:
            self.get_logger().error(f"Detachment error: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    collision_handler = PackageCollisionHandler()
    
    try:
        collision_handler.get_logger().info("🚀 Package collision handler running...")
        rclpy.spin(collision_handler)
    except KeyboardInterrupt:
        collision_handler.get_logger().info("🛑 Shutting down package collision handler...")
    finally:
        # Cleanup
        if collision_handler.package_attached:
            collision_handler.detach_package()
        
        collision_handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()