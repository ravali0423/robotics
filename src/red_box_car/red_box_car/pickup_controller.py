#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
import threading
import time


class PickupController(Node):
    def __init__(self):
        super().__init__('pickup_controller')
        
        # Publishers
        self.navigation_command_pub = self.create_publisher(String, '/navigation_command', 10)
        self.target_position_pub = self.create_publisher(Point, '/target_position', 10)
        self.pickup_status_pub = self.create_publisher(String, '/pickup_status', 10)
        
        # Subscribers
        self.pickup_command_sub = self.create_subscription(String, '/pickup_command', self.pickup_command_callback, 10)
        self.red_box_position_sub = self.create_subscription(Point, '/red_box_position', self.red_box_position_callback, 10)
        self.red_box_detected_sub = self.create_subscription(Bool, '/red_box_detected', self.red_box_detected_callback, 10)
        self.navigation_status_sub = self.create_subscription(String, '/navigation_status', self.navigation_status_callback, 10)
        
        # Internal state
        self.red_box_position = Point()
        self.red_box_detected = False
        self.navigation_status = "idle"
        self.pickup_active = False
        self.pickup_state = "idle"  # idle, navigating, approaching, picking_up, completed, failed
        self.lock = threading.Lock()
        
        # Parameters
        self.approach_distance = 0.5  # How close to get to the red box (meters)
        
        # State machine timer
        self.state_timer = self.create_timer(0.5, self.state_machine_update)
        
        self.get_logger().info("Pickup Controller started")
    
    def pickup_command_callback(self, msg):
        """Handle pickup commands"""
        command = msg.data.lower().strip()
        
        if command == "start" or command == "pickup":
            self.start_pickup_sequence()
        elif command == "stop" or command == "cancel":
            self.stop_pickup_sequence()
        elif command == "retry":
            self.retry_pickup()
        else:
            self.get_logger().warn(f"Unknown pickup command: {command}")
    
    def red_box_position_callback(self, msg):
        """Update red box position"""
        with self.lock:
            self.red_box_position = msg
    
    def red_box_detected_callback(self, msg):
        """Update red box detection status"""
        with self.lock:
            self.red_box_detected = msg.data
    
    def navigation_status_callback(self, msg):
        """Update navigation status"""
        with self.lock:
            self.navigation_status = msg.data
    
    def start_pickup_sequence(self):
        """Start the pickup sequence"""
        with self.lock:
            if not self.red_box_detected:
                self.get_logger().error("Cannot start pickup: Red box not detected!")
                self.publish_status("failed", "Red box not detected")
                return
            
            self.pickup_active = True
            self.pickup_state = "navigating"
            red_box_pos = self.red_box_position
        
        self.get_logger().info("Starting pickup sequence")
        self.publish_status("navigating", "Moving to red box location")
        
        # Calculate approach position (stop a bit before the red box)
        approach_position = Point()
        approach_position.x = red_box_pos.x
        approach_position.y = red_box_pos.y
        approach_position.z = 0.0
        
        # Publish target position and start navigation
        self.target_position_pub.publish(approach_position)
        
        nav_command = String()
        nav_command.data = "start"
        self.navigation_command_pub.publish(nav_command)
    
    def stop_pickup_sequence(self):
        """Stop the pickup sequence"""
        with self.lock:
            self.pickup_active = False
            self.pickup_state = "idle"
        
        # Stop navigation
        nav_command = String()
        nav_command.data = "stop"
        self.navigation_command_pub.publish(nav_command)
        
        self.publish_status("stopped", "Pickup sequence stopped")
        self.get_logger().info("Pickup sequence stopped")
    
    def retry_pickup(self):
        """Retry the pickup sequence"""
        self.get_logger().info("Retrying pickup sequence")
        self.start_pickup_sequence()
    
    def state_machine_update(self):
        """Update the pickup state machine"""
        with self.lock:
            if not self.pickup_active:
                return
            
            pickup_state = self.pickup_state
            nav_status = self.navigation_status
            red_box_detected = self.red_box_detected
        
        if pickup_state == "navigating":
            if nav_status == "goal_reached":
                self.transition_to_approaching()
            elif nav_status == "stopped" or nav_status == "failed":
                self.transition_to_failed("Navigation failed")
        
        elif pickup_state == "approaching":
            # In a real scenario, this would involve more complex approach logic
            # For now, we'll transition directly to picking up
            time.sleep(1)  # Brief pause
            self.transition_to_picking_up()
        
        elif pickup_state == "picking_up":
            # Simulate picking up action
            time.sleep(2)  # Simulate gripper action time
            self.transition_to_completed()
    
    def transition_to_approaching(self):
        """Transition to approaching state"""
        with self.lock:
            self.pickup_state = "approaching"
        
        self.publish_status("approaching", "Approaching red box")
        self.get_logger().info("Approaching red box")
        
        # Stop navigation as we're close enough
        nav_command = String()
        nav_command.data = "stop"
        self.navigation_command_pub.publish(nav_command)
    
    def transition_to_picking_up(self):
        """Transition to picking up state"""
        with self.lock:
            self.pickup_state = "picking_up"
        
        self.publish_status("picking_up", "Picking up red box")
        self.get_logger().info("Picking up red box...")
        
        # In a real implementation, this would activate a gripper or arm
        # For simulation, we just wait and then complete
    
    def transition_to_completed(self):
        """Transition to completed state"""
        with self.lock:
            self.pickup_active = False
            self.pickup_state = "completed"
        
        self.publish_status("completed", "Red box pickup completed successfully")
        self.get_logger().info("Pickup completed successfully!")
    
    def transition_to_failed(self, reason):
        """Transition to failed state"""
        with self.lock:
            self.pickup_active = False
            self.pickup_state = "failed"
        
        self.publish_status("failed", f"Pickup failed: {reason}")
        self.get_logger().error(f"Pickup failed: {reason}")
    
    def publish_status(self, status, message):
        """Publish pickup status"""
        status_msg = String()
        status_msg.data = f"{status}: {message}"
        self.pickup_status_pub.publish(status_msg)
    
    def get_pickup_status(self):
        """Get current pickup status"""
        with self.lock:
            return {
                'active': self.pickup_active,
                'state': self.pickup_state,
                'red_box_detected': self.red_box_detected,
                'navigation_status': self.navigation_status
            }


def main(args=None):
    rclpy.init(args=args)
    
    pickup_controller = PickupController()
    
    try:
        rclpy.spin(pickup_controller)
    except KeyboardInterrupt:
        pass
    finally:
        pickup_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()