#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 1.0   # m/s
        self.angular_speed = 1.0  # rad/s
        self.speed_increment = 0.1
        
        # Key bindings
        self.movement_bindings = {
            'w': (1, 0),    # forward
            's': (-1, 0),   # backward
            'a': (0, 1),    # turn left
            'd': (0, -1),   # turn right
            'q': (1, 1),    # forward + left
            'e': (1, -1),   # forward + right
            'z': (-1, 1),   # backward + left
            'c': (-1, -1),  # backward + right
        }
        
        self.speed_bindings = {
            'r': (1.1, 1.1),   # increase both speeds
            'f': (0.9, 0.9),   # decrease both speeds
            't': (1.1, 1.0),   # increase only linear speed
            'g': (0.9, 1.0),   # decrease only linear speed
            'y': (1.0, 1.1),   # increase only angular speed
            'h': (1.0, 0.9),   # decrease only angular speed
        }
        
        self.get_logger().info('Keyboard Teleop Node Started')
        self.print_usage()
        
        # Start keyboard control loop
        self.run_teleop()
    
    def print_usage(self):
        msg = """
Keyboard Teleop Control for Red Box Car
=======================================

Movement Commands:
    w    - Move Forward
    s    - Move Backward
    a    - Turn Left
    d    - Turn Right
    q    - Forward + Left
    e    - Forward + Right
    z    - Backward + Left
    c    - Backward + Right

Speed Control:
    r    - Increase both speeds
    f    - Decrease both speeds
    t    - Increase linear speed
    g    - Decrease linear speed
    y    - Increase angular speed
    h    - Decrease angular speed

Other:
    Space - Stop the car
    Ctrl-C - Quit

Current speeds:
    Linear:  {:.2f} m/s
    Angular: {:.2f} rad/s
        """.format(self.linear_speed, self.angular_speed)
        
        print(msg)
    
    def get_key(self):
        """Get a single keypress from stdin"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_twist(self, linear_x, angular_z):
        """Publish a Twist message with given linear and angular velocities"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z
        
        self.cmd_vel_pub.publish(twist)
    
    def run_teleop(self):
        """Main teleop control loop"""
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x03':  # Ctrl-C
                    break
                elif key == ' ':  # Space - stop
                    self.publish_twist(0.0, 0.0)
                    self.get_logger().info('Car stopped')
                elif key in self.movement_bindings:
                    linear_factor, angular_factor = self.movement_bindings[key]
                    linear_vel = linear_factor * self.linear_speed
                    angular_vel = angular_factor * self.angular_speed
                    
                    self.publish_twist(linear_vel, angular_vel)
                    self.get_logger().info(f'Moving: linear={linear_vel:.2f}, angular={angular_vel:.2f}')
                elif key in self.speed_bindings:
                    linear_factor, angular_factor = self.speed_bindings[key]
                    self.linear_speed *= linear_factor
                    self.angular_speed *= angular_factor
                    
                    # Limit speeds to reasonable ranges
                    self.linear_speed = max(0.1, min(5.0, self.linear_speed))
                    self.angular_speed = max(0.1, min(5.0, self.angular_speed))
                    
                    self.get_logger().info(f'Speed updated: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}')
                    # Stop the car after speed change
                    self.publish_twist(0.0, 0.0)
                elif key != '':
                    self.get_logger().warn(f'Unknown key: {key}')
                
        except Exception as e:
            self.get_logger().error(f'Error in teleop: {e}')
        finally:
            # Stop the car and restore terminal settings
            self.publish_twist(0.0, 0.0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        keyboard_teleop = KeyboardTeleop()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error starting keyboard teleop: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()