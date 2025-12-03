#!/usr/bin/env python3
"""
Mission log viewer and manager for warehouse robot.
"""

import os
import sys
from datetime import datetime

def list_mission_logs():
    """List all available mission log files."""
    log_dir = "/home/ravali/ros2_ws/mission_logs"
    
    if not os.path.exists(log_dir):
        print("❌ No mission logs directory found.")
        return []
    
    log_files = [f for f in os.listdir(log_dir) if f.endswith('.log')]
    
    if not log_files:
        print("📝 No mission logs found.")
        return []
    
    # Sort by creation time (newest first)
    log_files.sort(key=lambda f: os.path.getctime(os.path.join(log_dir, f)), reverse=True)
    
    print("📋 Available Mission Logs:")
    print("=" * 50)
    
    for i, log_file in enumerate(log_files, 1):
        log_path = os.path.join(log_dir, log_file)
        file_size = os.path.getsize(log_path)
        creation_time = datetime.fromtimestamp(os.path.getctime(log_path))
        
        print(f"{i}. {log_file}")
        print(f"   📅 Created: {creation_time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"   📏 Size: {file_size} bytes")
        print()
    
    return log_files

def view_latest_log():
    """View the latest mission log."""
    log_files = list_mission_logs()
    
    if not log_files:
        return
    
    latest_log = log_files[0]
    log_path = os.path.join("/home/ravali/ros2_ws/mission_logs", latest_log)
    
    print(f"📖 Viewing latest log: {latest_log}")
    print("=" * 60)
    
    try:
        with open(log_path, 'r') as f:
            content = f.read()
            print(content)
    except Exception as e:
        print(f"❌ Error reading log file: {e}")

def view_log_summary(log_file):
    """View a summary of a specific log file."""
    log_path = os.path.join("/home/ravali/ros2_ws/mission_logs", log_file)
    
    if not os.path.exists(log_path):
        print(f"❌ Log file not found: {log_file}")
        return
    
    print(f"📊 Mission Summary: {log_file}")
    print("=" * 60)
    
    try:
        with open(log_path, 'r') as f:
            lines = f.readlines()
        
        # Extract key information
        mission_start = None
        mission_end = None
        phases = []
        errors = []
        
        for line in lines:
            if "Mission start time:" in line:
                mission_start = line.split("Mission start time: ")[1].strip()
            elif "Mission end time:" in line:
                mission_end = line.split("Mission end time: ")[1].strip()
            elif "PHASE" in line:
                phases.append(line.split(" - INFO - ")[1].strip())
            elif "ERROR" in line or "❌" in line:
                errors.append(line.split(" - INFO - ")[1].strip())
        
        print(f"🚀 Mission Start: {mission_start}")
        print(f"🏁 Mission End: {mission_end}")
        print(f"📋 Total Lines: {len(lines)}")
        
        if phases:
            print("\n📍 Mission Phases:")
            for phase in phases:
                print(f"   • {phase}")
        
        if errors:
            print("\n❌ Errors/Issues:")
            for error in errors:
                print(f"   • {error}")
        else:
            print("\n✅ No errors found")
            
    except Exception as e:
        print(f"❌ Error reading log file: {e}")

def clean_old_logs(keep_count=10):
    """Clean old log files, keeping only the specified number."""
    log_dir = "/home/ravali/ros2_ws/mission_logs"
    
    if not os.path.exists(log_dir):
        print("❌ No mission logs directory found.")
        return
    
    log_files = [f for f in os.listdir(log_dir) if f.endswith('.log')]
    
    if len(log_files) <= keep_count:
        print(f"📝 Only {len(log_files)} logs found, no cleanup needed.")
        return
    
    # Sort by creation time (oldest first for deletion)
    log_files.sort(key=lambda f: os.path.getctime(os.path.join(log_dir, f)))
    
    files_to_delete = log_files[:-keep_count]
    
    print(f"🗑️  Deleting {len(files_to_delete)} old log files (keeping {keep_count} newest):")
    
    for log_file in files_to_delete:
        log_path = os.path.join(log_dir, log_file)
        try:
            os.remove(log_path)
            print(f"   ✅ Deleted: {log_file}")
        except Exception as e:
            print(f"   ❌ Failed to delete {log_file}: {e}")

def main():
    if len(sys.argv) < 2:
        print("🤖 Mission Log Manager")
        print("=" * 30)
        print("Available commands:")
        print("  list       - List all mission logs")
        print("  latest     - View latest mission log")
        print("  summary    - Show summary of latest log")
        print("  clean      - Clean old logs (keep 10 newest)")
        print("  clean N    - Clean old logs (keep N newest)")
        print("\nUsage: python3 mission_log_manager.py <command>")
        return
    
    command = sys.argv[1].lower()
    
    if command == "list":
        list_mission_logs()
    elif command == "latest":
        view_latest_log()
    elif command == "summary":
        log_files = list_mission_logs()
        if log_files:
            view_log_summary(log_files[0])
    elif command == "clean":
        keep_count = 10
        if len(sys.argv) > 2:
            try:
                keep_count = int(sys.argv[2])
            except ValueError:
                print("❌ Invalid number for keep count")
                return
        clean_old_logs(keep_count)
    else:
        print(f"❌ Unknown command: {command}")

if __name__ == "__main__":
    main()