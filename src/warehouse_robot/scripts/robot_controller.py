#!/usr/bin/env python3
"""
Enhanced warehouse robot controller with package pickup and delivery.
Supports autonomous navigation between waypoints and package manipulation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import sys
import os


class WarehouseRobotController(Node):
    def __init__(self):
        super().__init__('warehouse_robot_controller')
        
        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/model/warehouse_car/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/model/warehouse_car/odometry', self.odom_callback, 10)
        
        # Wait for publisher to be ready
        self.get_logger().info("⏳ Waiting for publisher to connect...")
        time.sleep(2.0)  # Give publisher time to connect
        self.get_logger().info(f"📡 Publisher subscriber count: {self.cmd_vel_publisher.get_subscription_count()}")
        
        # Robot state
        self.current_position = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.package_attached = False
        self.current_state = "idle"  # idle, moving_to_package, moving_to_destination, returning_to_start
        
        # Load waypoints
        self.waypoints = self.load_waypoints()
        
        # Navigation parameters
        self.linear_speed = 1.5
        self.angular_speed = 1.0
        self.position_tolerance = 0.3
        self.angle_tolerance = 0.1
        
        self.get_logger().info("🤖 Warehouse Robot Controller initialized!")
        self.get_logger().info(f"📍 Waypoints loaded: {len(self.waypoints)} points")
        
    def load_waypoints(self):
        """Load waypoints from file."""
        waypoints_file = "/home/ravali/ros2_ws/src/warehouse_robot/scripts/waypoints.txt"
        waypoints = {}
        
        try:
            with open(waypoints_file, 'r') as f:
                for line in f:
                    if '=' in line:
                        key, value = line.strip().split('=')
                        waypoints[key] = float(value)
            
            return {
                "start": {"x": waypoints.get("START_X", 0.0), "y": waypoints.get("START_Y", 0.0)},
                "package": {"x": waypoints.get("PACKAGE_X", 0.0), "y": waypoints.get("PACKAGE_Y", 0.0)},
                "destination": {"x": waypoints.get("DEST_X", 5.0), "y": waypoints.get("DEST_Y", 5.0)}
            }
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return {
                "start": {"x": 0.0, "y": 0.0},
                "package": {"x": 2.0, "y": 2.0},
                "destination": {"x": 5.0, "y": 5.0}
            }
    
    def odom_callback(self, msg):
        """Update robot position from odometry."""
        self.current_position["x"] = msg.pose.pose.position.x
        self.current_position["y"] = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        self.current_position["yaw"] = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )
        
        # Log position updates (less frequently)
        if hasattr(self, '_odom_count'):
            self._odom_count += 1
        else:
            self._odom_count = 1
        
        if self._odom_count % 50 == 0:  # Log every 50th message to avoid spam
            self.get_logger().info(f"📍 Odometry update: pos=({self.current_position['x']:.2f}, {self.current_position['y']:.2f}, yaw={self.current_position['yaw']:.2f})")
    
    def calculate_distance(self, target_x, target_y):
        """Calculate distance to target point."""
        dx = target_x - self.current_position["x"]
        dy = target_y - self.current_position["y"]
        return math.sqrt(dx*dx + dy*dy)
    
    def calculate_angle_to_target(self, target_x, target_y):
        """Calculate angle to target point."""
        dx = target_x - self.current_position["x"]
        dy = target_y - self.current_position["y"]
        return math.atan2(dy, dx)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def move_to_target(self, target_x, target_y):
        """Move robot to target position."""
        self.get_logger().info(f"🎯 Moving to ({target_x:.2f}, {target_y:.2f})")
        
        rate = self.create_rate(10)  # 10 Hz
        
        while True:
            # Calculate distance and angle to target
            distance = self.calculate_distance(target_x, target_y)
            
            if distance < self.position_tolerance:
                self.stop_robot()
                self.get_logger().info(f"✅ Reached target ({target_x:.2f}, {target_y:.2f})")
                break
            
            # Calculate angle to target
            target_angle = self.calculate_angle_to_target(target_x, target_y)
            angle_diff = self.normalize_angle(target_angle - self.current_position["yaw"])
            
            # Create velocity command
            twist = Twist()
            
            # Log current state for debugging
            self.get_logger().info(f"🔍 Current pos: ({self.current_position['x']:.2f}, {self.current_position['y']:.2f}, yaw: {self.current_position['yaw']:.2f})")
            self.get_logger().info(f"🎯 Target: ({target_x:.2f}, {target_y:.2f}), Distance: {distance:.2f}, Angle diff: {angle_diff:.2f}")
            
            # If we need to turn significantly, prioritize rotation
            if abs(angle_diff) > self.angle_tolerance:
                twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                twist.linear.x = 0.5 if abs(angle_diff) < 1.0 else 0.0  # Slow forward while turning
                self.get_logger().info(f"🔄 Turning: linear_x={twist.linear.x:.2f}, angular_z={twist.angular.z:.2f}")
            else:
                # Move forward
                twist.linear.x = min(self.linear_speed, distance * 2.0)  # Slow down when close
                twist.angular.z = angle_diff * 2.0  # Fine-tune direction
                self.get_logger().info(f"➡️ Moving forward: linear_x={twist.linear.x:.2f}, angular_z={twist.angular.z:.2f}")
            
            self.get_logger().info(f"📤 Publishing twist: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
            
            # Ensure message is published
            for i in range(3):  # Try publishing multiple times
                self.cmd_vel_publisher.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.01)  # Process any pending callbacks
            
            rate.sleep()
    
    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()
        self.get_logger().info("🛑 Stopping robot...")
        
        # Send stop command multiple times to ensure it's received
        for i in range(5):
            self.cmd_vel_publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.1)
        
        self.get_logger().info("✅ Stop command sent")
    
    def attach_package(self):
        """Simulate attaching package to robot."""
        self.get_logger().info("📦 Attaching package...")
        
        # Here you would normally use Gazebo services to attach the package
        # For now, we'll simulate it with a service call to move the package
        try:
            import subprocess
            package_x = self.current_position["x"]
            package_y = self.current_position["y"]
            package_z = 1.0  # Height on top of robot
            
            # Move package to robot position
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', f'name: "pickup_package", position: {{x: {package_x}, y: {package_y}, z: {package_z}}}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                self.package_attached = True
                self.get_logger().info("✅ Package attached successfully!")
            else:
                self.get_logger().error(f"❌ Failed to attach package: {result.stderr}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error attaching package: {e}")
    
    def detach_package(self):
        """Simulate detaching package from robot."""
        self.get_logger().info("📦 Detaching package...")
        
        try:
            import subprocess
            # Drop package at current location
            package_x = self.current_position["x"]
            package_y = self.current_position["y"]
            package_z = 0.15  # Ground level
            
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', f'name: "pickup_package", position: {{x: {package_x}, y: {package_y}, z: {package_z}}}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                self.package_attached = False
                self.get_logger().info("✅ Package delivered successfully!")
            else:
                self.get_logger().error(f"❌ Failed to detach package: {result.stderr}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error detaching package: {e}")
    
    def go_to_package(self):
        """Navigate to package location and pick it up."""
        if self.package_attached:
            self.get_logger().warn("📦 Package already attached!")
            return
        
        self.current_state = "moving_to_package"
        package_pos = self.waypoints["package"]
        
        self.get_logger().info(f"🚗 Going to pickup package at ({package_pos['x']:.2f}, {package_pos['y']:.2f})")
        self.move_to_target(package_pos["x"], package_pos["y"])
        
        # Attach package
        time.sleep(1.0)  # Small delay
        self.attach_package()
        self.current_state = "package_picked_up"
    
    def go_to_destination(self):
        """Navigate to destination and deliver package."""
        if not self.package_attached:
            self.get_logger().warn("📦 No package to deliver! Pick up package first.")
            return
        
        self.current_state = "moving_to_destination"
        dest_pos = self.waypoints["destination"]
        
        self.get_logger().info(f"🚗 Delivering package to ({dest_pos['x']:.2f}, {dest_pos['y']:.2f})")
        self.move_to_target(dest_pos["x"], dest_pos["y"])
        
        # Detach package
        time.sleep(1.0)  # Small delay
        self.detach_package()
        self.current_state = "package_delivered"
    
    def return_to_start(self):
        """Return to start position."""
        self.current_state = "returning_to_start"
        start_pos = self.waypoints["start"]
        
        self.get_logger().info(f"🚗 Returning to start position ({start_pos['x']:.2f}, {start_pos['y']:.2f})")
        self.move_to_target(start_pos["x"], start_pos["y"])
        self.current_state = "idle"
        self.get_logger().info("🏁 Mission complete! Robot ready for next task.")
    
    def run_full_mission(self):
        """Execute complete pickup and delivery mission."""
        self.get_logger().info("🚀 Starting full delivery mission!")
        
        self.go_to_package()
        time.sleep(2.0)
        
        self.go_to_destination()
        time.sleep(2.0)
        
        self.return_to_start()
    
    def print_status(self):
        """Print current robot status."""
        pos = self.current_position
        print(f"\n🤖 Robot Status:")
        print(f"   Position: ({pos['x']:.2f}, {pos['y']:.2f})")
        print(f"   State: {self.current_state}")
        print(f"   Package: {'📦 Attached' if self.package_attached else '📭 Not attached'}")
        print(f"   Waypoints:")
        for name, pos in self.waypoints.items():
            print(f"     {name}: ({pos['x']:.2f}, {pos['y']:.2f})")


def main():
    if len(sys.argv) < 2:
        print("🤖 Warehouse Robot Controller")
        print("=" * 40)
        print("Available commands:")
        print("  pickup     - Go to package location and pick it up")
        print("  deliver    - Go to destination and deliver package")
        print("  return     - Return to start position")
        print("  mission    - Execute full pickup and delivery mission")
        print("  status     - Show current robot status")
        print("  goto X Y   - Move to specific coordinates (e.g., goto 2.0 3.0)")
        print("\nUsage: ros2 run warehouse_robot robot_controller.py <command> [args]")
        return
    
    rclpy.init()
    controller = WarehouseRobotController()
    
    command = sys.argv[1].lower()
    
    try:
        if command == "pickup":
            controller.go_to_package()
        elif command == "deliver":
            controller.go_to_destination()
        elif command == "return":
            controller.return_to_start()
        elif command == "mission":
            controller.run_full_mission()
        elif command == "status":
            controller.print_status()
        elif command == "goto":
            if len(sys.argv) < 4:
                print("❌ Usage: goto <x> <y>")
                print("   Example: ros2 run warehouse_robot robot_controller.py goto 2.0 3.0")
                return
            try:
                target_x = float(sys.argv[2])
                target_y = float(sys.argv[3])
                controller.get_logger().info(f"🎯 Moving to coordinates ({target_x:.2f}, {target_y:.2f})")
                controller.move_to_target(target_x, target_y)
                controller.get_logger().info(f"✅ Successfully reached ({target_x:.2f}, {target_y:.2f})")
            except ValueError:
                print("❌ Invalid coordinates. Please use numbers (e.g., goto 2.0 3.0)")
        else:
            print(f"❌ Unknown command: {command}")
            return
            
    except KeyboardInterrupt:
        print("\n🛑 Operation interrupted by user")
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()