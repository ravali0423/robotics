#!/usr/bin/env python3
"""
Simple Keyboard Controller for Warehouse Robot

Alternative control method using keyboard input for robot movement.
Use WASD keys for movement, space to stop.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Create publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/model/warehouse_car/cmd_vel', 
            10
        )
        
        # Movement parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.8  # rad/s
        
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('🎮 Keyboard Controller initialized')
        self.print_instructions()
        
    def print_instructions(self):
        """Print control instructions"""
        instructions = """
🎮 WAREHOUSE ROBOT KEYBOARD CONTROLLER 🤖

Controls:
  W : Move Forward
  S : Move Backward  
  A : Turn Left
  D : Turn Right
  Q : Move Forward + Turn Left
  E : Move Forward + Turn Right
  Z : Move Backward + Turn Left
  C : Move Backward + Turn Right
  
  SPACE : Stop
  X : Emergency Stop & Exit

Current Speed Settings:
  Linear: {:.1f} m/s
  Angular: {:.1f} rad/s

Hold down keys for continuous movement!
Press Ctrl+C or X to exit.
        """.format(self.linear_speed, self.angular_speed)
        
        print(instructions)

    def get_key(self):
        """Get a single keypress"""
        tty.setraw(sys.stdin.fileno())
        
        # Check if there's input available
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1)
        else:
            key = ''
            
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def create_twist_message(self, linear_x=0.0, angular_z=0.0):
        """Create a Twist message with given velocities"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(angular_z)
        return twist

    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        """Publish velocity command to robot"""
        twist = self.create_twist_message(linear_x, angular_z)
        self.cmd_vel_publisher.publish(twist)
        
        # Log movement (only for non-zero velocities)
        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            self.get_logger().info(
                f'🚗 Moving: linear={linear_x:.2f} m/s, angular={angular_z:.2f} rad/s'
            )

    def stop_robot(self):
        """Stop the robot immediately"""
        self.publish_velocity(0.0, 0.0)
        self.get_logger().info('🛑 Robot stopped')

    def run_controller(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key().lower()
                
                if key == '':
                    # No key pressed, continue loop
                    continue
                elif key == 'w':
                    # Forward
                    self.publish_velocity(self.linear_speed, 0.0)
                elif key == 's':
                    # Backward
                    self.publish_velocity(-self.linear_speed, 0.0)
                elif key == 'a':
                    # Turn left
                    self.publish_velocity(0.0, self.angular_speed)
                elif key == 'd':
                    # Turn right
                    self.publish_velocity(0.0, -self.angular_speed)
                elif key == 'q':
                    # Forward + left
                    self.publish_velocity(self.linear_speed, self.angular_speed)
                elif key == 'e':
                    # Forward + right
                    self.publish_velocity(self.linear_speed, -self.angular_speed)
                elif key == 'z':
                    # Backward + left
                    self.publish_velocity(-self.linear_speed, self.angular_speed)
                elif key == 'c':
                    # Backward + right
                    self.publish_velocity(-self.linear_speed, -self.angular_speed)
                elif key == ' ':
                    # Stop
                    self.stop_robot()
                elif key == 'x':
                    # Exit
                    self.get_logger().info('🚪 Exiting keyboard controller...')
                    self.stop_robot()
                    break
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    # Unknown key
                    if ord(key) >= 32:  # Printable character
                        self.get_logger().warning(f'❓ Unknown key: {key}')
                
                # Small delay to prevent overwhelming the system
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_robot()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = KeyboardControllerNode()
        controller.run_controller()
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()