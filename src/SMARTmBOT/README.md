# SMARTmBOT Package

This is the main ROS2 package for the SMARTmBOT robot, featuring Gazebo simulation integration, autonomous navigation, and obstacle avoidance.

## Package Structure

- `src/` - Contains C++ source files
  - `smartmbot_node.cpp` - Main robot control node with obstacle avoidance
- `include/SMARTmBOT/` - Contains header files
- `launch/` - Contains launch files
  - `smartmbot_launch.py` - Complete simulation and robot launch
- `config/` - Contains configuration files
- `package.xml` - Package manifest
- `CMakeLists.txt` - CMake build configuration

## Features

- **Differential Drive Robot**: Two-wheeled robot with caster wheel
- **Lidar Sensor**: 360-degree laser scanner for obstacle detection
- **Obstacle Avoidance**: Autonomous navigation with basic obstacle avoidance
- **Gazebo Integration**: Full simulation environment
- **ROS2 Topics**: Standard ROS2 communication

## Related Packages

- `SMARTmBOT_description` - URDF/SDF robot models and descriptions
- `SMARTmBOT_gazebo` - Gazebo simulation worlds and launch files

## Building the Packages

To build all SMARTmBOT packages:

```bash
cd /home/ravali/ros2_ws
colcon build --packages-select SMARTmBOT SMARTmBOT_description SMARTmBOT_gazebo
```

## Running the Simulation

### Complete Simulation (Recommended)
Launch everything with one command:

```bash
source install/setup.bash
ros2 launch SMARTmBOT smartmbot_launch.py
```

### Manual Launch Steps

1. **Start Gazebo simulation:**
```bash
ros2 launch SMARTmBOT_gazebo smartmbot_simulation.launch.py
```

2. **Run robot control node:**
```bash
ros2 run SMARTmBOT smartmbot_node
```

3. **Manual robot control (optional):**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## ROS2 Topics

- `/cmd_vel` - Robot velocity commands (geometry_msgs/Twist)
- `/scan` - Lidar laser scan data (sensor_msgs/LaserScan)
- `/odom` - Robot odometry (nav_msgs/Odometry)
- `/smartmbot_status` - Robot status messages (std_msgs/String)

## Robot Behavior

The robot implements basic autonomous behavior:
- **Forward Movement**: Moves forward at 0.2 m/s when no obstacles detected
- **Obstacle Avoidance**: Stops and turns right when obstacles detected within 1 meter
- **Lidar Integration**: Uses 360-degree laser scan for obstacle detection

## Package Dependencies

### Core Dependencies
- `rclcpp` - ROS2 C++ client library
- `std_msgs` - Standard ROS2 message types
- `geometry_msgs` - Geometry-related messages (Twist)
- `sensor_msgs` - Sensor data messages (LaserScan)
- `nav_msgs` - Navigation messages (Odometry)

### Simulation Dependencies
- `SMARTmBOT_description` - Robot URDF/SDF models
- `SMARTmBOT_gazebo` - Gazebo simulation package
- `ros_gz_sim` - ROS2-Gazebo integration
- `ros_gz_bridge` - ROS2-Gazebo message bridge

## Troubleshooting

If you encounter issues:

1. **Make sure all packages are built:**
```bash
colcon build --packages-select SMARTmBOT SMARTmBOT_description SMARTmBOT_gazebo
```

2. **Source the workspace:**
```bash
source install/setup.bash
```

3. **Check topic connections:**
```bash
ros2 topic list
ros2 topic echo /scan
```

4. **Monitor robot status:**
```bash
ros2 topic echo /smartmbot_status
```