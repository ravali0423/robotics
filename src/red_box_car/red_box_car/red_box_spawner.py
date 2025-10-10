#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import time
import subprocess
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class RedBoxSpawner(Node):
    def __init__(self):
        super().__init__('red_box_spawner')
        
        # Publishers for red box position
        self.red_box_position_pub = self.create_publisher(Point, '/spawned_red_box_position', 10)
        
        # Get the path to the red box URDF file
        pkg_name = 'red_box_car'
        self.red_box_urdf_path = os.path.join(
            get_package_share_directory(pkg_name),
            'urdf',
            'red_box.urdf'
        )
        
        self.get_logger().info(f"Red box URDF path: {self.red_box_urdf_path}")
        
        # Store spawned position
        self.spawned_position = Point()
        
        # Wait a bit for Gazebo to be ready, then spawn the box
        self.timer = self.create_timer(3.0, self.spawn_red_box)
        self.spawned = False
    
    def spawn_red_box(self):
        if self.spawned:
            return
            
        # Generate random position on the ground plane
        # Keep it within a reasonable range around the origin
        x = random.uniform(-5.0, 5.0)
        y = random.uniform(-5.0, 5.0)
        z = 0.15  # Half the box height above ground
        
        # Generate random orientation (yaw)
        yaw = random.uniform(0, 0)
        
        self.get_logger().info(f"Spawning red box at position: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}")
        
        try:
            # Use gz sim command to spawn the red box
            cmd = [
                'gz', 'service', '-s', '/world/default/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'sdf_filename: "{self.red_box_urdf_path}", name: "red_box", pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{z: {yaw/2}, w: {(1-(yaw/2)**2)**0.5}}} }}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info("Successfully spawned red box!")
                
                # Store and publish the spawned position
                self.spawned_position.x = x
                self.spawned_position.y = y
                self.spawned_position.z = z
                
                # Publish the position for other nodes to use
                self.red_box_position_pub.publish(self.spawned_position)
                
                self.spawned = True
                self.timer.destroy()
                
                # Create a timer to periodically publish the position
                self.position_timer = self.create_timer(1.0, self.publish_position)
            else:
                self.get_logger().error(f"Failed to spawn red box: {result.stderr}")
                # Try alternative method using ros_gz_sim create
                self.try_alternative_spawn(x, y, z, yaw)
                
        except subprocess.TimeoutExpired:
            self.get_logger().error("Timeout while spawning red box")
            # Try alternative method
            self.try_alternative_spawn(x, y, z, yaw)
        except Exception as e:
            self.get_logger().error(f"Exception while spawning red box: {str(e)}")
            # Try alternative method
            self.try_alternative_spawn(x, y, z, yaw)
    
    def try_alternative_spawn(self, x, y, z, yaw):
        """Alternative spawning method using ros_gz_sim create"""
        try:
            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-file', self.red_box_urdf_path,
                '-name', 'red_box',
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
                '-Y', str(yaw)
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info("Successfully spawned red box using alternative method!")
                
                # Store and publish the spawned position
                self.spawned_position.x = x
                self.spawned_position.y = y
                self.spawned_position.z = z
                
                # Publish the position for other nodes to use
                self.red_box_position_pub.publish(self.spawned_position)
                
                self.spawned = True
                self.timer.destroy()
                
                # Create a timer to periodically publish the position
                self.position_timer = self.create_timer(1.0, self.publish_position)
            else:
                self.get_logger().error(f"Failed to spawn red box with alternative method: {result.stderr}")
                
        except Exception as e:
            self.get_logger().error(f"Exception with alternative spawn method: {str(e)}")
    
    def publish_position(self):
        """Periodically publish the red box position"""
        if self.spawned:
            self.red_box_position_pub.publish(self.spawned_position)


def main(args=None):
    rclpy.init(args=args)
    
    red_box_spawner = RedBoxSpawner()
    
    try:
        rclpy.spin(red_box_spawner)
    except KeyboardInterrupt:
        pass
    finally:
        red_box_spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()