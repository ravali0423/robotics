#!/usr/bin/env python3
"""
Simple utility to display current waypoints.
"""

def read_waypoints():
    """Read and display current waypoints."""
    waypoints_file = "/home/ravali/ros2_ws/src/warehouse_robot/scripts/waypoints.txt"
    
    try:
        with open(waypoints_file, 'r') as f:
            print("📍 Current Waypoints:")
            print("=" * 30)
            for line in f:
                key, value = line.strip().split('=')
                if 'START' in key:
                    print(f"🟢 {key}: {value}")
                else:
                    print(f"🔴 {key}: {value}")
        print("=" * 30)
    except FileNotFoundError:
        print("❌ No waypoints file found. Run generate_waypoints.py first.")
    except Exception as e:
        print(f"❌ Error reading waypoints: {e}")


if __name__ == "__main__":
    read_waypoints()