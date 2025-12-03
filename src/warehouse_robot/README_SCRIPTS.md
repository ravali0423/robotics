# Warehouse Robot Scripts Documentation

## Current Scripts Overview

This directory contains the finalized scripts for the warehouse robot project. All test and experimental files have been cleaned up.

### Core Control Scripts

1. **`web_joystick_controller.py`** ⭐ **MAIN JOYSTICK CONTROLLER**
   - **Purpose**: Complete web-based joystick control with embedded HTML interface
   - **Features**: 
     - Drag-drop circular joystick with real-time control
     - Automatic browser opening at http://localhost:8080/joystick.html
     - HTTP server with ROS2 integration
     - Velocity feedback and robot stopping on release
   - **Usage**: `python3 scripts/web_joystick_controller.py`
   - **Status**: ✅ Fully functional and actively used

2. **`manual_robot_controller.py`** 📋 **BACKUP CONTROLLER**
   - **Purpose**: Terminal-based interactive robot control
   - **Features**: WASD key controls, speed adjustment, real-time feedback
   - **Usage**: `python3 scripts/manual_robot_controller.py`
   - **Status**: ✅ Working backup solution

### Mission & Navigation Scripts

3. **`robot_controller.py`** 🤖 **AUTONOMOUS NAVIGATION**
   - **Purpose**: Autonomous waypoint navigation and mission execution
   - **Features**: Path planning, obstacle avoidance, mission logging
   - **Status**: ✅ Core functionality complete

4. **`car_controller.py`** 🚗 **BASIC VEHICLE CONTROL**
   - **Purpose**: Low-level vehicle movement functions
   - **Usage**: Imported by other controllers
   - **Status**: ✅ Stable library

### Utility Scripts

5. **`generate_waypoints.py`** 📍 **WAYPOINT GENERATION**
   - **Purpose**: Interactive waypoint creation for missions
   - **Features**: Click-to-place waypoints in simulation
   - **Status**: ✅ Functional

6. **`generate_new_waypoints.py`** 📍 **SIMPLE WAYPOINT CREATOR**
   - **Purpose**: Quick waypoint generation utility
   - **Status**: ✅ Utility script

7. **`show_waypoints.py`** 👁️ **WAYPOINT VISUALIZATION**
   - **Purpose**: Display current waypoints
   - **Status**: ✅ Helper utility

8. **`mission_log_manager.py`** 📊 **MISSION LOGGING**
   - **Purpose**: Log and track mission progress
   - **Status**: ✅ Logging utility

### Configuration Files

9. **`waypoints.txt`** 📋 **CURRENT WAYPOINTS**
   - **Purpose**: Stores generated waypoints for missions
   - **Format**: START_X=0.0, START_Y=0.0, etc.

## Quick Start

### For Manual Robot Control:
```bash
# Terminal 1: Start Gazebo simulation
ros2 launch warehouse_robot warehouse_with_joystick.launch.py

# Terminal 2: Start web joystick controller
cd /home/ravali/ros2_ws/src/warehouse_robot
python3 scripts/web_joystick_controller.py
```

### For Backup Manual Control:
```bash
python3 scripts/manual_robot_controller.py
```

## File Status Summary
- ✅ **11 functional scripts** - All core functionality preserved
- 🗑️ **3 test scripts removed** - Cleaned up experimental code
- 🎮 **1 primary joystick solution** - Web-based controller
- 📋 **1 backup solution** - Manual terminal controller

## Notes
- All test scripts (`test_robot_movement.py`, `simple_robot_test.py`, `keyboard_controller.py`) have been removed
- The web joystick controller is the primary user interface
- Manual controller serves as a reliable backup option
- All core robot functionality remains intact