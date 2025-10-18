# SMARTmBOT Gazebo Package

This package provides Gazebo simulation capabilities for the SMARTmBOT robot.

## Package Contents

### Launch Files
- `smartmbot_simulation.launch.py` - Complete simulation setup
- `smartmbot_bringup.launch.py` - Robot nodes and teleoperation

### World Files  
- `smartmbot_world.sdf` - Test environment with obstacles

### Configuration
- `bridge_config.yaml` - ROS-Gazebo topic bridge configuration

## Features

### Simulation Environment
- **Physics**: 1ms timestep realistic physics simulation
- **Lighting**: Directional sunlight with shadows
- **Ground**: Large flat plane for navigation testing
- **Obstacles**: Various geometric obstacles for navigation testing
  - Red box (1x1x1m) 
  - Green rectangular barrier (0.5x2x1m)
  - Blue cylinder (0.5m radius, 1m height)

### ROS-Gazebo Integration
- **Command Interface**: `/cmd_vel` topic for robot control
- **Sensor Data**: `/scan` topic for lidar data  
- **Odometry**: `/odom` topic for robot pose
- **Joint States**: `/joint_states` for robot joint positions

### Robot Capabilities in Simulation
- **Differential Drive**: Realistic wheel physics and control
- **Lidar Sensing**: GPU-accelerated 360° laser scanning
- **Collision Detection**: Accurate collision geometry
- **Inertial Dynamics**: Proper mass and inertia properties

## Usage

### Launch Complete Simulation
```bash
# Start Gazebo with SMARTmBOT robot
ros2 launch SMARTmBOT_gazebo smartmbot_simulation.launch.py
```

### Launch with Custom World
```bash
# Use different world file
ros2 launch SMARTmBOT_gazebo smartmbot_simulation.launch.py world:=/path/to/your/world.sdf
```

### Control the Robot
```bash
# Manual teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or launch the autonomous behavior
ros2 launch SMARTmBOT_gazebo smartmbot_bringup.launch.py
```

## ROS Topics

### Commands (ROS → Gazebo)
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

### Sensor Data (Gazebo → ROS)  
- `/scan` (sensor_msgs/LaserScan) - Lidar measurements
- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/joint_states` (sensor_msgs/JointState) - Joint positions

## Dependencies

### Gazebo Integration
- `ros_gz_sim` - ROS2 Gazebo simulation interface
- `ros_gz_bridge` - Message translation between ROS2 and Gazebo

### Robot Description
- `SMARTmBOT_description` - Robot URDF/SDF models
- `robot_state_publisher` - Publishes robot transforms
- `joint_state_publisher` - Joint state management

### Utilities
- `xacro` - XML macro processing for URDF files

## Simulation Performance

### Recommended System Requirements
- **CPU**: Multi-core processor (4+ cores recommended)
- **GPU**: Dedicated graphics card for GPU lidar simulation
- **RAM**: 4GB+ available memory
- **OS**: Ubuntu 22.04 with ROS2 Humble

### Performance Tips
- Use `gz sim -v4` for verbose output to debug performance issues
- Monitor CPU/GPU usage during simulation
- Reduce lidar update rate if experiencing performance issues
- Use headless mode for automated testing: `gz sim --headless-rendering`

## Troubleshooting

### Common Issues

1. **Gazebo won't start:**
```bash
# Check if Gazebo is already running
ps aux | grep gz
killall gz  # Kill existing instances
```

2. **Robot doesn't spawn:**
```bash  
# Check robot description
ros2 topic echo /robot_description
```

3. **No sensor data:**
```bash
# Verify ROS-Gazebo bridge
ros2 topic list | grep -E "(scan|odom|cmd_vel)"
```

4. **Poor performance:**
```bash
# Run with reduced graphics
gz sim --render-engine ogre --headless-rendering
```

### Debug Commands
```bash
# List Gazebo topics
gz topic -l

# Monitor Gazebo topic data  
gz topic -e /scan

# Check Gazebo model info
gz model -m smartmbot -i
```