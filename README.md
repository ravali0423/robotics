# UR5 Robot Arm Simulation

Professional Universal Robots UR5 6-DOF industrial robotic arm simulation for ROS 2 Jazzy.

## Features

- 🏭 **Professional UR5 Model**: Uses official Universal Robots URDF description
- 🎮 **Interactive Control**: Joint State Publisher GUI for manual joint control  
- 📊 **3D Visualization**: RViz2 with pre-configured views
- 🚀 **One-Click Launch**: Automated setup and launch script
- 📦 **Ultra-Minimal**: Only 6 files, uses system packages for dependencies

## Setup

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

### Git Repository Setup

This workspace includes a comprehensive `.gitignore` file that excludes:
- ROS 2 build artifacts (`build/`, `install/`, `log/`)
- IDE configurations (`.vscode/`, `.idea/`)
- Python cache files (`__pycache__/`, `*.pyc`)
- System files (`.DS_Store`, `Thumbs.db`)
- ROS data files (`*.bag`, `*.db3`, `*.mcap`)

Only essential source files are tracked in git.

## Quick Start

1. **Run the simulation**:
   ```bash
   cd ~/ros2_ws
   ./launch_arm_sim.sh
   ```

The script will automatically:
- Install all required dependencies
- Build the workspace if needed
- Launch the UR5 simulation

## Manual Setup (Alternative)

If you prefer manual setup:

```bash
# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Launch simulation
ros2 launch arm_simulation view_arm.launch.py
```

## What You'll See

- **Joint State Publisher GUI**: Control panel with 6 joint sliders
- **RViz2**: 3D visualization of the UR5 robot arm
- **Professional UR5 Model**: Realistic industrial robot appearance

## Requirements

- ROS 2 Jazzy
- Ubuntu 24.04 (recommended)
- Dependencies automatically installed by the script

## Package Structure

```
src/
└── arm_simulation/                    # Minimal launch package
    ├── CMakeLists.txt                # Build configuration
    ├── package.xml                   # Package metadata  
    ├── launch/view_arm.launch.py     # Launch file for UR5 simulation
    └── rviz/ur5_arm.rviz             # RViz configuration

Root Files:
├── .gitignore                        # Git ignore file (excludes build artifacts)
├── README.md                         # This documentation
└── launch_arm_sim.sh                 # One-click launch script
```

### External Dependencies (Auto-Managed)

- **Universal Robots ROS2 Description**
  - **Source**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
  - **Package**: `ros-jazzy-ur-description`
  - **Contains**: Official URDF models for all UR robot series (UR3, UR5, UR10, UR16, UR20)
  - **Installation**: Automatically handled by launch script

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

## Controls

Use the **Joint State Publisher GUI** window to control the robot:
- 6 sliders for shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
- Real-time movement in RViz2
- Realistic joint limits matching actual UR5 specifications

## Troubleshooting

If you only see joint axes but no robot:
1. Check RViz2 → Displays → RobotModel is enabled
2. Verify "Visual Enabled" is checked
3. Try zooming out or resetting the view
4. Ensure `/robot_description` topic is active

---

**Enjoy exploring the professional UR5 robot simulation!** 🤖