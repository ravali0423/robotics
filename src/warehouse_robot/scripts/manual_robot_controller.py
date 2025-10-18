#!/usr/bin/env python3
"""
Simple Manual Robot Controller

This script provides simple manual control of the warehouse robot.
Run this script and use the keyboard commands while watching the robot in Gazebo.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class ManualRobotController(Node):
    def __init__(self):
        super().__init__('manual_robot_controller')
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/model/warehouse_car/cmd_vel', 
            10
        )
        
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        
        self.get_logger().info('🎮 Manual Robot Controller Started!')
        self.print_instructions()
        
    def print_instructions(self):
        instructions = """
🤖 WAREHOUSE ROBOT MANUAL CONTROLLER 🎮

Commands:
  w : Move Forward
  s : Move Backward
  a : Turn Left  
  d : Turn Right
  q : Forward + Left
  e : Forward + Right
  z : Backward + Left
  c : Backward + Right
  
  x : STOP Robot
  + : Increase Speed
  - : Decrease Speed
  
  h : Show this help
  exit : Quit controller

Current Speeds: Linear={:.1f} m/s, Angular={:.1f} rad/s

Type command and press Enter:
        """.format(self.linear_speed, self.angular_speed)
        
        print(instructions)
        
    def move_robot(self, linear_x=0.0, angular_z=0.0):
        """Send movement command to robot"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        
        self.cmd_vel_publisher.publish(twist)
        
        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            self.get_logger().info(f'🚗 Moving: linear={linear_x:.2f}, angular={angular_z:.2f}')
        else:
            self.get_logger().info('🛑 Stopped')
            
    def run_controller(self):
        """Run the interactive controller"""
        try:
            while rclpy.ok():
                try:
                    command = input("Robot> ").strip().lower()
                    
                    if command == 'w':
                        self.move_robot(self.linear_speed, 0.0)
                    elif command == 's':
                        self.move_robot(-self.linear_speed, 0.0)
                    elif command == 'a':
                        self.move_robot(0.0, self.angular_speed)
                    elif command == 'd':
                        self.move_robot(0.0, -self.angular_speed)
                    elif command == 'q':
                        self.move_robot(self.linear_speed, self.angular_speed)
                    elif command == 'e':
                        self.move_robot(self.linear_speed, -self.angular_speed)
                    elif command == 'z':
                        self.move_robot(-self.linear_speed, self.angular_speed)
                    elif command == 'c':
                        self.move_robot(-self.linear_speed, -self.angular_speed)
                    elif command == 'x':
                        self.move_robot(0.0, 0.0)
                    elif command == '+':
                        self.linear_speed = min(1.0, self.linear_speed + 0.1)
                        self.angular_speed = min(2.0, self.angular_speed + 0.1)
                        self.get_logger().info(f'Speed increased: Linear={self.linear_speed:.1f}, Angular={self.angular_speed:.1f}')
                    elif command == '-':
                        self.linear_speed = max(0.1, self.linear_speed - 0.1)
                        self.angular_speed = max(0.1, self.angular_speed - 0.1)
                        self.get_logger().info(f'Speed decreased: Linear={self.linear_speed:.1f}, Angular={self.angular_speed:.1f}')
                    elif command == 'h':
                        self.print_instructions()
                    elif command == 'exit':
                        self.get_logger().info('🚪 Exiting controller...')
                        break
                    elif command == '':
                        continue
                    else:
                        self.get_logger().warning(f'❓ Unknown command: {command}. Type "h" for help.')
                        
                    # Small delay and spin
                    rclpy.spin_once(self, timeout_sec=0.01)
                    
                except KeyboardInterrupt:
                    break
                except EOFError:
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            self.move_robot(0.0, 0.0)  # Stop robot
            self.get_logger().info('🛑 Robot stopped. Goodbye!')

def main():
    rclpy.init()
    
    try:
        controller = ManualRobotController()
        controller.run_controller()
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()