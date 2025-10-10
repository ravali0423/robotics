#!/usr/bin/env python3
"""
Random waypoint generator for warehouse robot navigation.
Generates start (green) and destination (red) points avoiding obstacles.
"""

import random
import math
import xml.etree.ElementTree as ET
from pathlib import Path


class WarehouseWaypointGenerator:
    def __init__(self):
        # World boundaries (with safety margin)
        self.world_min_x = -9.0
        self.world_max_x = 9.0
        self.world_min_y = -9.0
        self.world_max_y = 9.0
        
        # Obstacle areas (shelf positions with safety margin)
        self.obstacles = [
            # Shelf 1: center at (8, 0), size 1x4, with safety margin
            {"min_x": 7.0, "max_x": 9.0, "min_y": -2.5, "max_y": 2.5},
            # Shelf 2: center at (-8, 0), size 1x4, with safety margin  
            {"min_x": -9.0, "max_x": -7.0, "min_y": -2.5, "max_y": 2.5}
        ]
        
        # Minimum distance between start and destination
        self.min_distance = 3.0
        
    def is_point_in_obstacle(self, x, y):
        """Check if a point is inside any obstacle area."""
        for obstacle in self.obstacles:
            if (obstacle["min_x"] <= x <= obstacle["max_x"] and 
                obstacle["min_y"] <= y <= obstacle["max_y"]):
                return True
        return False
    
    def generate_valid_point(self):
        """Generate a random point that's not in any obstacle."""
        max_attempts = 100
        for _ in range(max_attempts):
            x = random.uniform(self.world_min_x, self.world_max_x)
            y = random.uniform(self.world_min_y, self.world_max_y)
            
            if not self.is_point_in_obstacle(x, y):
                return x, y
                
        # Fallback if no valid point found
        return 0.0, 0.0
    
    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def generate_waypoints(self):
        """Generate start and destination points with minimum distance."""
        max_attempts = 100
        
        for _ in range(max_attempts):
            start_x, start_y = self.generate_valid_point()
            dest_x, dest_y = self.generate_valid_point()
            
            distance = self.calculate_distance(start_x, start_y, dest_x, dest_y)
            
            if distance >= self.min_distance:
                return {
                    "start": {"x": start_x, "y": start_y},
                    "destination": {"x": dest_x, "y": dest_y}
                }
        
        # Fallback waypoints if generation fails
        return {
            "start": {"x": -5.0, "y": -5.0},
            "destination": {"x": 5.0, "y": 5.0}
        }
    
    def create_marker_sdf(self, name, x, y, color):
        """Create SDF XML for a waypoint marker."""
        if color == "green":
            ambient = "0.0 1.0 0.0 1.0"
            diffuse = "0.0 1.0 0.0 1.0"
        else:  # red
            ambient = "1.0 0.0 0.0 1.0"
            diffuse = "1.0 0.0 0.0 1.0"
            
        marker_sdf = f'''
    <model name="{name}_marker">
      <static>true</static>
      <pose>{x} {y} 0.1 0 0 0</pose>
      <link name="marker_link">
        <visual name="marker_visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{ambient}</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>0.2 0.2 0.2 1.0</specular>
            <emissive>0.1 0.1 0.1 1.0</emissive>
          </material>
        </visual>
        <collision name="marker_collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>'''
        return marker_sdf
    
    def update_world_file(self, world_file_path, waypoints):
        """Update the SDF world file with waypoint markers."""
        try:
            # Read the current world file
            tree = ET.parse(world_file_path)
            root = tree.getroot()
            
            # Find the world element
            world = root.find('world')
            if world is None:
                print("Error: Could not find world element in SDF file")
                return False
            
            # Remove existing waypoint markers if they exist
            for model in world.findall('model'):
                if model.get('name') and ('_marker' in model.get('name')):
                    world.remove(model)
            
            # Create start marker (green)
            start_marker = ET.fromstring(self.create_marker_sdf(
                "start", 
                waypoints["start"]["x"], 
                waypoints["start"]["y"], 
                "green"
            ))
            world.append(start_marker)
            
            # Create destination marker (red)
            dest_marker = ET.fromstring(self.create_marker_sdf(
                "destination", 
                waypoints["destination"]["x"], 
                waypoints["destination"]["y"], 
                "red"
            ))
            world.append(dest_marker)
            
            # Write the updated file
            tree.write(world_file_path, encoding='utf-8', xml_declaration=True)
            
            print(f"✅ Updated waypoints:")
            print(f"   🟢 Start: ({waypoints['start']['x']:.2f}, {waypoints['start']['y']:.2f})")
            print(f"   🔴 Destination: ({waypoints['destination']['x']:.2f}, {waypoints['destination']['y']:.2f})")
            
            return True
            
        except Exception as e:
            print(f"Error updating world file: {e}")
            return False
    
    def save_waypoints_to_file(self, waypoints, output_file):
        """Save waypoints to a file for use by launch script."""
        try:
            with open(output_file, 'w') as f:
                f.write(f"START_X={waypoints['start']['x']:.3f}\n")
                f.write(f"START_Y={waypoints['start']['y']:.3f}\n")
                f.write(f"DEST_X={waypoints['destination']['x']:.3f}\n")
                f.write(f"DEST_Y={waypoints['destination']['y']:.3f}\n")
            return True
        except Exception as e:
            print(f"Error saving waypoints file: {e}")
            return False


def main():
    # Generate waypoints
    generator = WarehouseWaypointGenerator()
    waypoints = generator.generate_waypoints()
    
    # Update world file
    world_file = Path(__file__).parent.parent / "worlds" / "warehouse_world.sdf"
    if generator.update_world_file(world_file, waypoints):
        print("✅ World file updated successfully!")
    else:
        print("❌ Failed to update world file")
        return
    
    # Save waypoints for launch script
    waypoints_file = Path(__file__).parent / "waypoints.txt"
    if generator.save_waypoints_to_file(waypoints, waypoints_file):
        print("✅ Waypoints saved for launch script!")
    else:
        print("❌ Failed to save waypoints file")


if __name__ == "__main__":
    main()