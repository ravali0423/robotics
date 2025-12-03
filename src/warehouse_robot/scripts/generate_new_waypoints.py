#!/usr/bin/env python3
"""
Script to generate NEW random waypoints.
Run this manually when you want new waypoint locations.
The launch file will use existing waypoints, not generate new ones.
"""

import subprocess
import sys
import os

def main():
    print("🎯 Generating NEW random waypoints...")
    
    # Get the directory of this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    generate_script = os.path.join(script_dir, "generate_waypoints.py")
    
    try:
        subprocess.run(["python3", generate_script], check=True)
        print("✅ NEW waypoints generated successfully!")
        print("🚀 You can now launch the simulation with: ros2 launch warehouse_robot custom_warehouse.launch.py")
    except Exception as e:
        print(f"❌ Failed to generate waypoints: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()