#!/bin/bash

# Professional UR5 Robot Arm Simulation Launch Script
# This script sets up and launches the Universal Robots UR5 6-DOF industrial robotic arm
# from a clean workspace to full visualization with interactive control

echo "🤖 Starting Professional UR5 Robot Arm Simulation..."
echo "📍 Workspace: $(pwd)"

# Change to workspace directory
cd ~/ros2_ws

# Check if we need to build the workspace
if [ ! -d "install" ]; then
    echo "🏗️  Building workspace from scratch..."
    
    # Install system dependencies including UR description
    echo "📦 Installing UR5 robot package and dependencies..."
    sudo apt update > /dev/null 2>&1
    sudo apt install -y ros-jazzy-ur-description > /dev/null 2>&1
    
    # Install other dependencies
    echo "📦 Installing workspace dependencies..."
    rosdep install --from-paths src --ignore-src -r -y
    
    # Build the workspace
    echo "🔨 Building packages..."
    colcon build --symlink-install
    
    if [ $? -eq 0 ]; then
        echo "✅ Build completed successfully!"
    else
        echo "❌ Build failed! Please check the errors above."
        exit 1
    fi
else
    echo "✅ Workspace already built, using existing installation"
fi

# Source the ROS 2 setup
echo "🔧 Sourcing ROS 2 workspace..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch the UR5 simulation
echo "🚀 Launching UR5 robot arm simulation..."
echo "   - Robot State Publisher: Publishing UR5 model"
echo "   - Joint State Publisher GUI: Interactive joint control"
echo "   - RViz2: 3D visualization"
echo ""
echo "💡 Use the Joint State Publisher GUI sliders to move the UR5 joints!"
echo "   🏭 This is a professional Universal Robots UR5 industrial arm"
echo "   📐 6 degrees of freedom with realistic joint limits"
echo "   Press Ctrl+C to stop the simulation"
echo ""

ros2 launch arm_simulation view_arm.launch.py