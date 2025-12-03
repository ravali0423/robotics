# 🤖 Robotics Simulation Workspace

A comprehensive ROS2 workspace featuring three exciting robot simulations:
1. **Professional UR5 Industrial Arm** - 6-DOF industrial robotic arm
2. **Robotic Hand with Finger Counting** - 5-finger hand that displays finger counts (0-5)
3. **Warehouse Robot Drop Test** - iRobot Create3-based warehouse robot with physics simulation

## 🎯 Projects Overview

### 1. UR5 Robot Arm Simulation
- 🏭 **Professional UR5 Model**: Uses official Universal Robots URDF description
- 🎮 **Interactive Control**: Joint State Publisher GUI for manual joint control  
- 📊 **3D Visualization**: RViz2 with pre-configured views
- 🚀 **One-Click Launch**: Automated setup and launch script

### 2. Robotic Hand Finger Counting
- ✋ **5-Finger Hand**: Realistic hand with thumb, index, middle, ring, and pinky
- 🔢 **Finger Counting**: Show 0-5 fingers based on user commands
- 🎮 **Real-time Control**: Command fingers via ROS2 topics
- 📊 **3D Visualization**: Watch finger movements in RViz

### 3. Warehouse Robot Drop Test
- 🏢 **Amazon-style Warehouse Robot**: iRobot Create3-based autonomous robot
- 📦 **Realistic Warehouse Environment**: Complete with shelving, walls, and proper lighting
- ⬇️ **Physics Drop Test**: Robot spawns at height and drops with realistic collision detection
- 🛡️ **Static Ground Plane**: Properly configured collision detection that keeps the ground stable
- 🎮 **Gazebo Simulation**: Full physics simulation with interactive controlsation Workspace

A comprehensive ROS2 workspace featuring two exciting robot simulations:
1. **Professional UR5 Industrial Arm** - 6-DOF industrial robotic arm
2. **Robotic Hand with Finger Counting** - 5-finger hand that displays finger counts (0-5)

## 🎯 Projects Overview

### 1. UR5 Robot Arm Simulation
- 🏭 **Professional UR5 Model**: Uses official Universal Robots URDF description
- 🎮 **Interactive Control**: Joint State Publisher GUI for manual joint control  
- 📊 **3D Visualization**: RViz2 with pre-configured views
- 🚀 **One-Click Launch**: Automated setup and launch script

### 2. Robotic Hand Finger Counting
- �️ **5-Finger Hand**: Realistic hand with thumb, index, middle, ring, and pinky
- 🔢 **Finger Counting**: Show 0-5 fingers based on user commands
- 🎮 **Real-time Control**: Command fingers via ROS2 topics
- 📊 **3D Visualization**: Watch finger movements in RViz

## 📦 Installation & Setup

### Prerequisites
- **ROS2 Jazzy** (Ubuntu 24.04 recommended)
- **Build tools**: `colcon`, `rosdep`
- **Git** for cloning repositories

### Clone or Download This Workspace

```bash
# Option 1: Clone if this is a git repository
git clone <repository-url> ~/ros2_ws
cd ~/ros2_ws

# Option 2: Create workspace manually
mkdir -p ~/ros2_ws/src
# Copy the arm_simulation folder to ~/ros2_ws/src/
# Copy launch_arm_sim.sh and .gitignore to ~/ros2_ws/
```

### Build the Workspace

```bash
# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Install additional packages for robotic hand and warehouse robot
sudo apt update
sudo apt install ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-rviz2 \
                 ros-jazzy-xacro \
                 ros-jazzy-ur-description \
                 ros-jazzy-irobot-create-description \
                 ros-jazzy-irobot-create-gz-sim \
                 ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Option 3: Warehouse Robot Drop Test

1. **Launch the warehouse robot simulation**:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch irobot_create_gz_bringup create3_gz.launch.py robot_name:=warehouse_robot z:=2.0
   ```

2. **Control the warehouse robot** (in a new terminal):
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   
   # Move forward
   ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{
     header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
     twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}
   }" --once
   
   # Turn left
   ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{
     header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
     twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}
   }" --once
   
   # Stop robot
   ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{
     header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
     twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}
   }" --once
   ```

### Option 1: Robotic Hand Finger Counting

1. **Launch the robotic hand simulation**:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch arm_simulation robotic_hand.launch.py
   ros2 launch arm_simulation speech_to_gesture.launch.py
   ```

2. **Control finger count** (in a new terminal):
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   
   # Show 1 finger
   ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 1" --once
   
   # Show 2 fingers
   ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 2" --once
   
   # Show 3 fingers
   ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 3" --once
   
   # Show 4 fingers
   ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 4" --once
   
   # Show 5 fingers (open hand)
   ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 5" --once
   
   # Close hand (fist)
   ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 0" --once
   ```

## 📁 Project Structure

```
~/ros2_ws/
├── src/
│   ├── arm_simulation/                    # Main robotics package
│   │   ├── CMakeLists.txt                # Build configuration
│   │   ├── package.xml                   # Package dependencies
│   │   ├── launch/
│   │   │   ├── view_arm.launch.py        # UR5 arm simulation
│   │   │   └── robotic_hand.launch.py    # Robotic hand simulation
│   │   ├── urdf/
│   │   │   └── robotic_hand.urdf.xacro   # Hand robot description
│   │   ├── scripts/
│   │   │   ├── hand_controller.py        # Hand controller node
│   │   │   └── test_hand.py              # Test script
│   │   └── rviz/
│   │       ├── ur5_arm.rviz              # UR5 RViz config
│   │       └── robotic_hand.rviz         # Hand RViz config
│   └── warehouse_robot/                  # Warehouse robot package
│       ├── CMakeLists.txt                # Build configuration
│       ├── package.xml                   # Package dependencies
│       ├── launch/
│       │   ├── warehouse_simulation.launch.py  # Main simulation launch
│       │   └── simple_warehouse_sim.launch.py  # Simple launch
│       ├── urdf/
│       │   └── warehouse_robot.urdf.xacro      # Robot description
│       ├── worlds/
│       │   └── warehouse_world.sdf             # Warehouse world
│       ├── config/
│       │   └── control.yaml                    # Robot control config
│       └── rviz/
│           └── warehouse_robot.rviz            # Warehouse RViz config
├── .gitignore                            # Git ignore file
├── README.md                             # This documentation
└── launch_arm_sim.sh                     # UR5 launch script
```

### Git Repository Setup

This workspace includes a comprehensive `.gitignore` file that excludes:
- ROS 2 build artifacts (`build/`, `install/`, `log/`)
- IDE configurations (`.vscode/`, `.idea/`)
- Python cache files (`__pycache__/`, `*.pyc`)
- System files (`.DS_Store`, `Thumbs.db`)
- ROS data files (`*.bag`, `*.db3`, `*.mcap`)

Only essential source files are tracked in git.

## 🔧 Advanced Usage

### Monitor Robotic Hand Status
```bash
# Check active topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Check hand controller logs
ros2 node info /hand_controller

# Monitor finger count commands
ros2 topic echo /finger_count
```

### UR5 Arm Manual Control
```bash
# Launch UR5 with manual control
ros2 launch arm_simulation view_arm.launch.py

# Use Joint State Publisher GUI to control 6 joints:
# - shoulder_pan, shoulder_lift, elbow
# - wrist_1, wrist_2, wrist_3
```

### Custom Testing Script for Hand
```bash
# Run automated finger counting demo
cd ~/ros2_ws
source install/setup.bash
python3 src/arm_simulation/scripts/test_hand.py
```

## 🎯 What You'll See

### UR5 Robot Arm Simulation
- **Joint State Publisher GUI**: Control panel with 6 joint sliders
- **RViz2**: 3D visualization of the UR5 robot arm
- **Professional UR5 Model**: Realistic industrial robot appearance

### Robotic Hand Simulation
- **3D Hand Model**: Realistic 5-finger robotic hand
- **Real-time Finger Control**: Watch fingers move based on commands
- **Smooth Animations**: Natural finger movements between configurations
- **Joint Visualization**: See all finger joints and their movements

### Warehouse Robot Drop Test
- **Realistic Warehouse Environment**: Complete depot with shelving, walls, and realistic textures
- **iRobot Create3 Model**: Professional warehouse robot with sensors and collision detection
- **Physics Drop Test**: Watch the robot fall from 2 meters height and bounce realistically
- **Interactive Controls**: Use Gazebo's built-in teleop controls or ROS2 commands
- **Ground Collision**: Observe realistic collision detection with the static ground plane

## Dependencies

### Core Dependencies

- **`ros-jazzy-ur-description`** - Official Universal Robots UR5 model package
  - **GitHub**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
  - **Manual Installation**: `sudo apt install ros-jazzy-ur-description`
  - **Auto-installed** by launch script

- **Standard ROS 2 Packages** (auto-installed):
  - `robot_state_publisher` - Robot state publishing
  - `joint_state_publisher_gui` - Interactive joint control
  - `rviz2` - 3D visualization

### Manual Dependency Installation

If you want to install dependencies manually:

```bash
# Update package list
sudo apt update

# Install UR5 robot description package
sudo apt install ros-jazzy-ur-description

# Install workspace dependencies via rosdep
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Install additional ROS 2 packages if needed
sudo apt install ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-rviz2
```

### System Requirements

- **ROS 2 Jazzy** (Ubuntu 24.04 recommended)
- **Ubuntu 24.04 LTS** or compatible
- **Git** for cloning repositories
- **Build tools**: `colcon`, `rosdep`

## 🐛 Troubleshooting

### Common Issues and Solutions

**Problem:** `Package 'arm_simulation' not found`
```bash
# Solution: Make sure workspace is built and sourced
cd ~/ros2_ws
colcon build --packages-select arm_simulation
source install/setup.bash
```

**Problem:** `No executable found` (for robotic hand)
```bash
# Solution: Ensure scripts are executable
chmod +x ~/ros2_ws/src/arm_simulation/scripts/hand_controller.py
chmod +x ~/ros2_ws/src/arm_simulation/scripts/test_hand.py
```

**Problem:** RViz doesn't show the robot
```bash
# Solution: Check robot_description topic
ros2 topic echo /robot_description --once

# Make sure robot_state_publisher is running
ros2 node list | grep robot_state_publisher
```

**Problem:** Hand doesn't respond to finger commands
```bash
# Check if hand_controller is running
ros2 node list | grep hand_controller

# Verify topic is published correctly
ros2 topic info /finger_count

# Check for error messages
ros2 node info /hand_controller
```

**Problem:** Only see joint axes but no robot in UR5 simulation
1. Check RViz2 → Displays → RobotModel is enabled
2. Verify "Visual Enabled" is checked
3. Try zooming out or resetting the view
4. Ensure `/robot_description` topic is active

**Warning:** "KDL does not support root link with inertia"
- This is a harmless warning and doesn't affect functionality
- The simulation will work correctly despite this message

### Getting Help
```bash
# List all available topics
ros2 topic list

# Get topic information
ros2 topic info /finger_count

# Monitor topic data
ros2 topic echo /finger_count

# Check running nodes
ros2 node list

# Get node information
ros2 node info /hand_controller
```

## 🎯 Next Steps & Project Ideas

### Robotic Hand Extensions
- Experiment with different finger combinations
- Create custom gesture sequences
- Add voice control integration
- Implement computer vision for gesture recognition
- Create automated finger counting games
- Add haptic feedback simulation

### UR5 Arm Extensions
- Add trajectory planning and execution
- Implement pick-and-place operations
- Create waypoint-based movement
- Add end-effector attachments
- Integrate motion planning frameworks

### Combined Projects
- Mount the robotic hand on the UR5 arm
- Create complete manipulation scenarios
- Implement object grasping simulations
- Add sensor integration (cameras, force sensors)

## 📚 Learning Resources

- **ROS2 Documentation**: https://docs.ros.org/en/jazzy/
- **Universal Robots ROS2**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
- **URDF Tutorials**: http://wiki.ros.org/urdf/Tutorials
- **RViz User Guide**: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html

---

**Happy robotics simulation! 🤖🖐️👍**



Craete a new package called warehouse_robot. Use an existing car urdf file and place it on top of a ground plane in the world. The car has to drop from a height onto the plane and the ground should have collision detection and should not fall down in the world ans should stay static in the world. I want to use Gazebo sim