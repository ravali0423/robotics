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
import logging
from datetime import datetime


class WarehouseRobotController(Node):
    def __init__(self, enable_file_logging=False):
        super().__init__('warehouse_robot_controller')
        
        # Setup file logging if enabled
        self.file_logging_enabled = enable_file_logging
        self.log_file = None
        if enable_file_logging:
            self.setup_file_logging()
        
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
        self.package_offset = None  # Offset for package following
        self.current_state = "idle"  # idle, moving_to_package, moving_to_destination, returning_to_start
        
        # Load waypoints
        self.waypoints = self.load_waypoints()
        
        # Navigation parameters - simplified for better control
        self.linear_speed = 0.8  # Slower for better control
        self.angular_speed = 0.6  # Slower turning
        self.position_tolerance = 0.5  # Smaller tolerance for precise stopping
        self.angle_tolerance = 0.2  # Not used in new logic, but kept for compatibility
        
        self.get_logger().info("🤖 Warehouse Robot Controller initialized!")
        self.get_logger().info(f"📍 Waypoints loaded: {len(self.waypoints)} points")
        
    def setup_file_logging(self):
        """Setup file logging for mission logs."""
        # Create logs directory if it doesn't exist
        log_dir = "/home/ravali/ros2_ws/mission_logs"
        os.makedirs(log_dir, exist_ok=True)
        
        # Create log filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"warehouse_robot_mission_{timestamp}.log"
        log_path = os.path.join(log_dir, log_filename)
        
        # Setup file logger
        self.file_logger = logging.getLogger('mission_logger')
        self.file_logger.setLevel(logging.INFO)
        
        # Remove any existing handlers
        for handler in self.file_logger.handlers[:]:
            self.file_logger.removeHandler(handler)
        
        # Create file handler
        file_handler = logging.FileHandler(log_path)
        file_handler.setLevel(logging.INFO)
        
        # Create formatter
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        
        # Add handler to logger
        self.file_logger.addHandler(file_handler)
        
        self.log_file = log_path
        self.log_to_file("🚀 Mission logging started")
        self.log_to_file(f"📝 Log file: {log_path}")
        self.get_logger().info(f"📝 Mission logs will be saved to: {log_path}")
        
    def log_to_file(self, message):
        """Log message to file if file logging is enabled."""
        if self.file_logging_enabled and hasattr(self, 'file_logger'):
            self.file_logger.info(message)
        
    def enhanced_log(self, message):
        """Log message to both ROS logger and file if enabled."""
        self.get_logger().info(message)
        self.log_to_file(message)
        
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
        # Store previous position for change detection
        prev_x = self.current_position["x"]
        prev_y = self.current_position["y"]
        
        self.current_position["x"] = msg.pose.pose.position.x
        self.current_position["y"] = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        self.current_position["yaw"] = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )
        
        # Detect if position actually changed
        position_changed = abs(self.current_position["x"] - prev_x) > 0.001 or abs(self.current_position["y"] - prev_y) > 0.001
        
        # Log position updates (less frequently)
        if hasattr(self, '_odom_count'):
            self._odom_count += 1
        else:
            self._odom_count = 1
        
        if self._odom_count % 50 == 0 or position_changed:  # Log every 50th message or when position changes
            self.get_logger().info(f"📍 Odometry update: pos=({self.current_position['x']:.2f}, {self.current_position['y']:.2f}, yaw={self.current_position['yaw']:.2f}) {'📈 MOVED' if position_changed else '📍 STATIC'}")
    
    def calculate_distance(self, target_x, target_y):
        """Calculate distance to target point."""
        dx = target_x - self.current_position["x"]
        dy = target_y - self.current_position["y"]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Debug: Log the calculation
        self.get_logger().debug(f"📏 Distance calc: target=({target_x:.2f},{target_y:.2f}), current=({self.current_position['x']:.2f},{self.current_position['y']:.2f}), distance={distance:.3f}")
        
        return distance
    
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
        self.enhanced_log(f"🎯 Moving to ({target_x:.2f}, {target_y:.2f})")
        self.log_to_file(f"Navigation started to target: ({target_x:.2f}, {target_y:.2f})")
        
        rate = self.create_rate(1)  # 1 Hz - send commands every 1 second
        max_iterations = 60  # 600 seconds at 1Hz - reasonable limit
        iteration_count = 0
        last_distance = float('inf')
        stuck_counter = 0
        
        while rclpy.ok() and iteration_count < max_iterations:
            iteration_count += 1

            self.log_to_file(f"Iteration: {iteration_count} - rclpy.ok()={rclpy.ok()}, max_iterations={max_iterations}")
                
            
            # Process ROS callbacks to get latest odometry
            rclpy.spin_once(self, timeout_sec=0.1)  # Longer timeout to ensure we get updates
            
            # Update package position if attached (make it follow the robot)
            self.update_package_position()
            
            # Calculate distance and angle to target
            distance = self.calculate_distance(target_x, target_y)
            
            self.log_to_file(f"Distance check: {distance:.3f} < {self.position_tolerance} = {distance < self.position_tolerance}")
            
            # Simple proximity detection - stop immediately when close to target
            if distance < self.position_tolerance:
                self.enhanced_log(f"🎯 Close to target! Distance: {distance:.3f}")
                self.log_to_file(f"Target reached! Final distance: {distance:.3f}")
                self.stop_robot()
                self.enhanced_log(f"✅ Reached target ({target_x:.2f}, {target_y:.2f}) - Final distance: {distance:.3f}")
                return  # Exit function completely
            
            # Check if we're making progress (anti-stuck mechanism)
            if abs(distance - last_distance) < 0.01:
                stuck_counter += 1
                if stuck_counter > 5:  # 5 seconds at 1Hz
                    self.enhanced_log("🚨 Robot appears stuck, trying recovery maneuver...")
                    self.log_to_file("Robot stuck - executing recovery maneuver")
                    # Recovery: small random movement
                    recovery_twist = Twist()
                    recovery_twist.linear.x = 0.2
                    recovery_twist.angular.z = 0.5
                    for _ in range(5):
                        self.cmd_vel_publisher.publish(recovery_twist)
                        time.sleep(0.1)
                    stuck_counter = 0
            else:
                stuck_counter = 0
            last_distance = distance
            
            # Calculate angle to target
            target_angle = self.calculate_angle_to_target(target_x, target_y)
            angle_diff = self.normalize_angle(target_angle - self.current_position["yaw"])
            
            # Create velocity command
            twist = Twist()
            
            # Log current state for debugging
            self.log_to_file(f"Current pos: ({self.current_position['x']:.2f}, {self.current_position['y']:.2f}, yaw: {self.current_position['yaw']:.2f})")
            self.log_to_file(f"Target: ({target_x:.2f}, {target_y:.2f}), Distance: {distance:.2f}, Angle diff: {angle_diff:.2f}")
            
            # Simplified and more robust navigation logic
            if abs(angle_diff) > 1.0:  # Need significant turning (>57 degrees)
                # Pure rotation - stop and turn
                twist.linear.x = 0.0
                twist.angular.z = 0.3 if angle_diff > 0 else -0.3  # Slower, controlled turning
                self.log_to_file(f"Turning only: angular_z={twist.angular.z:.2f}")
            else:
                # Good direction - move forward with minor corrections
                # Constant speed for simplicity
                twist.linear.x = 0.5  # Constant moderate speed
                twist.angular.z = angle_diff * 0.2  # Gentle steering
                self.log_to_file(f"Moving forward: linear_x={twist.linear.x:.2f}, angular_z={twist.angular.z:.2f}")
            
            self.log_to_file(f"Publishing twist: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
            
            # Publish the command
            self.cmd_vel_publisher.publish(twist)
            
            self.log_to_file(f"End of iteration {iteration_count}, going to sleep for 1 second...")
            
            # Simple time.sleep instead of rate.sleep to avoid hanging
            time.sleep(1.0)
            self.log_to_file(f"Woke up from sleep, starting iteration {iteration_count + 1}")
        
        # If we exit the loop, log why
        self.log_to_file(f"Exited loop: rclpy.ok()={rclpy.ok()}, iteration_count={iteration_count}, max_iterations={max_iterations}")
        
        # Safety: Always stop at the end if we exit the loop without reaching target
        self.enhanced_log(f"⚠️  Movement timed out after {max_iterations} iterations. Final distance: {self.calculate_distance(target_x, target_y):.2f}")
        self.log_to_file(f"Navigation timeout - final distance: {self.calculate_distance(target_x, target_y):.2f}")
        self.stop_robot()
        
        # Force stop for safety
        for _ in range(10):
            stop_twist = Twist()
            self.cmd_vel_publisher.publish(stop_twist)
            time.sleep(0.1)
    
    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()  # All values default to 0.0
        self.get_logger().info("🛑 Stopping robot...")
        
        # Send stop command multiple times to ensure it's received
        for i in range(10):
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.05)  # Small delay between commands
        
        self.get_logger().info("✅ Stop command sent")
    
    def attach_package(self):
        """Attach package to robot by positioning it on top."""
        self.enhanced_log("📦 Attaching package...")
        
        try:
            import subprocess
            
            # Calculate position on top of robot
            package_x = self.current_position["x"]
            package_y = self.current_position["y"] 
            package_z = 0.6  # Height on top of robot (robot is ~0.4m high)
            
            self.log_to_file(f"Attempting to attach package at robot position: ({package_x:.2f}, {package_y:.2f}, {package_z:.2f})")
            
            # Move package to robot position
            move_cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', f'name: "pickup_package", position: {{x: {package_x}, y: {package_y}, z: {package_z}}}'
            ]
            
            result = subprocess.run(move_cmd, capture_output=True, text=True)
            if result.returncode == 0:
                self.package_attached = True
                self.enhanced_log("✅ Package positioned on robot!")
                self.log_to_file("Package successfully attached to robot")
                
                # Start following the robot by storing initial relative position
                self.package_offset = {"x": 0.0, "y": 0.0, "z": 0.6}
                
            else:
                self.enhanced_log(f"❌ Failed to move package: {result.stderr}")
                self.log_to_file(f"Package attachment failed: {result.stderr}")
                
        except Exception as e:
            self.enhanced_log(f"❌ Error attaching package: {e}")
            self.log_to_file(f"Package attachment error: {str(e)}")
    
    def update_package_position(self):
        """Update package position to follow the robot if attached."""
        if not self.package_attached:
            return
            
        try:
            import subprocess
            
            # Calculate new package position relative to robot
            package_x = self.current_position["x"] + self.package_offset["x"]
            package_y = self.current_position["y"] + self.package_offset["y"]
            package_z = self.package_offset["z"]
            
            # Move package to follow robot
            move_cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',  # Shorter timeout for frequent updates
                '--req', f'name: "pickup_package", position: {{x: {package_x}, y: {package_y}, z: {package_z}}}'
            ]
            
            subprocess.run(move_cmd, capture_output=True, text=True)
            
        except Exception as e:
            # Don't spam errors for package following
            pass
    
    def detach_package(self):
        """Detach package from robot by dropping it at current location."""
        self.enhanced_log("📦 Detaching package...")
        
        try:
            import subprocess
            
            # Drop package at current location
            package_x = self.current_position["x"]
            package_y = self.current_position["y"]
            package_z = 0.15  # Ground level
            
            self.log_to_file(f"Attempting to detach package at location: ({package_x:.2f}, {package_y:.2f}, {package_z:.2f})")
            
            drop_cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', f'name: "pickup_package", position: {{x: {package_x}, y: {package_y}, z: {package_z}}}'
            ]
            
            result = subprocess.run(drop_cmd, capture_output=True, text=True)
            if result.returncode == 0:
                self.package_attached = False
                self.package_offset = None  # Clear offset
                self.enhanced_log("✅ Package delivered successfully!")
                self.log_to_file("Package successfully detached and delivered")
            else:
                self.enhanced_log(f"❌ Failed to drop package: {result.stderr}")
                self.log_to_file(f"Package detachment failed: {result.stderr}")
                # Still mark as detached for mission logic
                self.package_attached = False
                self.package_offset = None
                
        except Exception as e:
            self.enhanced_log(f"❌ Error detaching package: {e}")
            self.log_to_file(f"Package detachment error: {str(e)}")
            # Still mark as detached for mission logic
            self.package_attached = False
            self.package_offset = None
    
    def go_to_package(self):
        """Navigate to package location and pick it up."""
        if self.package_attached:
            self.enhanced_log("📦 Package already attached!")
            return
        
        self.current_state = "moving_to_package"
        package_pos = self.waypoints["package"]
        
        self.enhanced_log(f"🚗 Going to pickup package at ({package_pos['x']:.2f}, {package_pos['y']:.2f})")
        self.log_to_file(f"Starting navigation to package location: ({package_pos['x']:.2f}, {package_pos['y']:.2f})")
        
        self.move_to_target(package_pos["x"], package_pos["y"])
        
        # Attach package
        time.sleep(1.0)  # Small delay
        self.attach_package()
        self.current_state = "package_picked_up"
        self.enhanced_log("✅ Package pickup phase completed")
    
    def go_to_destination(self):
        """Navigate to destination and deliver package."""
        if not self.package_attached:
            self.enhanced_log("📦 No package to deliver! Pick up package first.")
            return
        
        self.current_state = "moving_to_destination"
        dest_pos = self.waypoints["destination"]
        
        self.enhanced_log(f"🚗 Delivering package to ({dest_pos['x']:.2f}, {dest_pos['y']:.2f})")
        self.log_to_file(f"Starting navigation to destination: ({dest_pos['x']:.2f}, {dest_pos['y']:.2f})")
        
        self.move_to_target(dest_pos["x"], dest_pos["y"])
        
        # Detach package
        time.sleep(1.0)  # Small delay
        self.detach_package()
        self.current_state = "package_delivered"
        self.enhanced_log("✅ Package delivery phase completed")
    
    def return_to_start(self):
        """Return to start position."""
        self.current_state = "returning_to_start"
        start_pos = self.waypoints["start"]
        
        self.enhanced_log(f"🚗 Returning to start position ({start_pos['x']:.2f}, {start_pos['y']:.2f})")
        self.log_to_file(f"Starting navigation to start position: ({start_pos['x']:.2f}, {start_pos['y']:.2f})")
        
        self.move_to_target(start_pos["x"], start_pos["y"])
        self.current_state = "idle"
        self.enhanced_log("🏁 Return to start phase completed")
    
    def run_full_mission(self):
        """Execute complete pickup and delivery mission."""
        self.enhanced_log("🚀 Starting full delivery mission!")
        
        # Log mission start details
        self.log_to_file("=" * 60)
        self.log_to_file("MISSION START")
        self.log_to_file("=" * 60)
        start_time = datetime.now()
        self.log_to_file(f"Mission start time: {start_time.strftime('%Y-%m-%d %H:%M:%S')}")
        
        # Log waypoints
        for name, pos in self.waypoints.items():
            self.log_to_file(f"Waypoint {name}: ({pos['x']:.2f}, {pos['y']:.2f})")
        
        try:
            # Phase 1: Go to package
            self.enhanced_log("📍 PHASE 1: Going to package location")
            self.go_to_package()
            time.sleep(2.0)
            
            # Phase 2: Deliver package
            self.enhanced_log("📍 PHASE 2: Delivering package to destination")
            self.go_to_destination()
            time.sleep(2.0)
            
            # Phase 3: Return to start
            self.enhanced_log("📍 PHASE 3: Returning to start position")
            self.return_to_start()
            
            # Mission completed successfully
            end_time = datetime.now()
            mission_duration = end_time - start_time
            self.log_to_file("=" * 60)
            self.log_to_file("MISSION COMPLETED SUCCESSFULLY")
            self.log_to_file("=" * 60)
            self.log_to_file(f"Mission end time: {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
            self.log_to_file(f"Total mission duration: {mission_duration}")
            self.enhanced_log("🏁 Mission completed successfully!")
            
        except Exception as e:
            # Mission failed
            end_time = datetime.now()
            mission_duration = end_time - start_time
            self.log_to_file("=" * 60)
            self.log_to_file("MISSION FAILED")
            self.log_to_file("=" * 60)
            self.log_to_file(f"Mission end time: {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
            self.log_to_file(f"Mission duration before failure: {mission_duration}")
            self.log_to_file(f"Error: {str(e)}")
            self.enhanced_log(f"❌ Mission failed: {str(e)}")
            raise
    
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
        print("  stop       - Stop the robot immediately")
        print("\nUsage: ros2 run warehouse_robot robot_controller.py <command> [args]")
        return
    
    rclpy.init()
    
    command = sys.argv[1].lower()
    
    # Enable file logging for mission command
    enable_logging = (command == "mission")
    controller = WarehouseRobotController(enable_file_logging=enable_logging)
    
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
        elif command == "stop":
            controller.stop_robot()
            controller.enhanced_log("🛑 Robot stopped by user command")
        elif command == "goto":
            if len(sys.argv) < 4:
                print("❌ Usage: goto <x> <y>")
                print("   Example: ros2 run warehouse_robot robot_controller.py goto 2.0 3.0")
                return
            try:
                target_x = float(sys.argv[2])
                target_y = float(sys.argv[3])
                controller.enhanced_log(f"🎯 Moving to coordinates ({target_x:.2f}, {target_y:.2f})")
                controller.move_to_target(target_x, target_y)
                controller.enhanced_log(f"✅ Successfully reached ({target_x:.2f}, {target_y:.2f})")
            except ValueError:
                print("❌ Invalid coordinates. Please use numbers (e.g., goto 2.0 3.0)")
        else:
            print(f"❌ Unknown command: {command}")
            return
            
    except KeyboardInterrupt:
        print("\n🛑 Operation interrupted by user")
        if enable_logging:
            controller.log_to_file("Mission interrupted by user (Ctrl+C)")
    finally:
        controller.stop_robot()
        if enable_logging:
            controller.log_to_file("🏁 Mission logging ended")
            if hasattr(controller, 'log_file') and controller.log_file:
                print(f"\n📝 Mission logs saved to: {controller.log_file}")
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()