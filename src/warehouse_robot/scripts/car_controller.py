#!/usr/bin/env python3
"""
Simple car controller for warehouse robot.
Use this script to control the car with simple commands.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time


class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.publisher = self.create_publisher(Twist, '/model/warehouse_car/cmd_vel', 10)
        self.get_logger().info('Car Controller initialized!')
        
    def move_forward(self, speed=1.0, duration=2.0):
        """Move car forward"""
        msg = Twist()
        msg.linear.x = speed
        self.publish_for_duration(msg, duration)
        
    def move_backward(self, speed=1.0, duration=2.0):
        """Move car backward"""
        msg = Twist()
        msg.linear.x = -speed
        self.publish_for_duration(msg, duration)
        
    def turn_left(self, angular_speed=1.0, duration=1.0):
        """Turn car left"""
        msg = Twist()
        msg.angular.z = angular_speed
        self.publish_for_duration(msg, duration)
        
    def turn_right(self, angular_speed=1.0, duration=1.0):
        """Turn car right"""
        msg = Twist()
        msg.angular.z = -angular_speed
        self.publish_for_duration(msg, duration)
        
    def move_forward_left(self, linear_speed=0.5, angular_speed=0.5, duration=2.0):
        """Move forward while turning left"""
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.publish_for_duration(msg, duration)
        
    def move_forward_right(self, linear_speed=0.5, angular_speed=0.5, duration=2.0):
        """Move forward while turning right"""
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = -angular_speed
        self.publish_for_duration(msg, duration)
        
    def stop(self):
        """Stop the car"""
        msg = Twist()  # All zeros
        self.publisher.publish(msg)
        self.get_logger().info('Car stopped!')
        
    def publish_for_duration(self, msg, duration):
        """Publish message for specified duration"""
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            rate.sleep()
            
        self.stop()  # Always stop after movement


def main():
    rclpy.init()
    controller = CarController()
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run warehouse_robot car_controller.py <command>")
        print("Commands:")
        print("  forward [speed] [duration]")
        print("  backward [speed] [duration]") 
        print("  left [angular_speed] [duration]")
        print("  right [angular_speed] [duration]")
        print("  forward_left [linear_speed] [angular_speed] [duration]")
        print("  forward_right [linear_speed] [angular_speed] [duration]")
        print("  stop")
        print("  demo")
        return
        
    command = sys.argv[1].lower()
    
    try:
        if command == 'forward':
            speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
            controller.get_logger().info(f'Moving forward at {speed} m/s for {duration}s')
            controller.move_forward(speed, duration)
            
        elif command == 'backward':
            speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
            controller.get_logger().info(f'Moving backward at {speed} m/s for {duration}s')
            controller.move_backward(speed, duration)
            
        elif command == 'left':
            angular_speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
            controller.get_logger().info(f'Turning left at {angular_speed} rad/s for {duration}s')
            controller.turn_left(angular_speed, duration)
            
        elif command == 'right':
            angular_speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
            controller.get_logger().info(f'Turning right at {angular_speed} rad/s for {duration}s')
            controller.turn_right(angular_speed, duration)
            
        elif command == 'forward_left':
            linear_speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
            angular_speed = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
            duration = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
            controller.get_logger().info(f'Moving forward-left for {duration}s')
            controller.move_forward_left(linear_speed, angular_speed, duration)
            
        elif command == 'forward_right':
            linear_speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
            angular_speed = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
            duration = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
            controller.get_logger().info(f'Moving forward-right for {duration}s')
            controller.move_forward_right(linear_speed, angular_speed, duration)
            
        elif command == 'stop':
            controller.stop()
            
        elif command == 'demo':
            controller.get_logger().info('Running demo sequence...')
            controller.move_forward(1.0, 2.0)
            time.sleep(1)
            controller.turn_right(1.0, 1.0)
            time.sleep(1)
            controller.move_backward(0.5, 1.0)
            time.sleep(1)
            controller.turn_left(1.0, 1.0)
            time.sleep(1)
            controller.move_forward_left(0.5, 0.5, 2.0)
            controller.get_logger().info('Demo completed!')
            
        else:
            controller.get_logger().error(f'Unknown command: {command}')
            
    except ValueError:
        controller.get_logger().error('Invalid number format in arguments')
    except Exception as e:
        controller.get_logger().error(f'Error: {e}')
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()