# 🤖 Robotics Simulation Workspace

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

# Install additional packages for robotic hand
sudo apt update
sudo apt install ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-rviz2 \
                 ros-jazzy-xacro \
                 ros-jazzy-ur-description

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## 🚀 Quick Start

### Option 1: UR5 Robot Arm Simulation

1. **Run the UR5 simulation**:
   ```bash
   cd ~/ros2_ws
   ./launch_arm_sim.sh
   ```

The script will automatically install dependencies, build workspace, and launch the UR5 simulation.

### Option 2: Robotic Hand Finger Counting

1. **Launch the robotic hand simulation**:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch arm_simulation robotic_hand.launch.py
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

## 🎮 Robotic Hand Topic Commands

### Finger Count Topic: `/finger_count`
**Message Type:** `std_msgs/msg/Int32`

| Command | Fingers Shown | Description |
|---------|---------------|-------------|
| `ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 0" --once` | 0 | Closed fist |
| `ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 1" --once` | 1 | Index finger only |
| `ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 2" --once` | 2 | Index + middle |
| `ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 3" --once` | 3 | Index + middle + ring |
| `ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 4" --once` | 4 | All except thumb |
| `ros2 topic pub /finger_count std_msgs/msg/Int32 "data: 5" --once` | 5 | Open hand (all fingers) |

### Quick Test Sequence
```bash
# Test all finger counts in sequence
for i in {0..5}; do
  echo "Showing $i finger(s)..."
  ros2 topic pub /finger_count std_msgs/msg/Int32 "data: $i" --once
  sleep 2
done
```

## 📁 Project Structure

```
~/ros2_ws/
├── src/
│   └── arm_simulation/                    # Main robotics package
│       ├── CMakeLists.txt                # Build configuration
│       ├── package.xml                   # Package dependencies
│       ├── launch/
│       │   ├── view_arm.launch.py        # UR5 arm simulation
│       │   └── robotic_hand.launch.py    # Robotic hand simulation
│       ├── urdf/
│       │   └── robotic_hand.urdf.xacro   # Hand robot description
│       ├── scripts/
│       │   ├── hand_controller.py        # Hand controller node
│       │   └── test_hand.py              # Test script
│       └── rviz/
│           ├── ur5_arm.rviz              # UR5 RViz config
│           └── robotic_hand.rviz         # Hand RViz config
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



cd /home/ravali/ros2_ws
source install/setup.bash
ros2 launch warehouse_robot warehouse_simulation.launch.py


# Move to predefined locations
ros2 run warehouse_robot move_robot center
ros2 run warehouse_robot move_robot corner1
ros2 run warehouse_robot move_robot wall1
ros2 run warehouse_robot move_robot front

# Move to custom coordinates
ros2 run warehouse_robot move_robot '{"x": 3.0, "y": -2.0}'
ros2 run warehouse_robot move_robot '{"x": -5.0, "y": 4.0}'