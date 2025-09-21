#!/bin/bash

# UR5 Randomize Test Script
echo "🤖 Testing UR5 Randomize Functionality..."
echo ""

# Check if ROS2 is sourced
if ! command -v ros2 &> /dev/null; then
    echo "❌ Error: ROS2 not found. Please source your ROS2 installation."
    exit 1
fi

# Check if workspace is sourced
if ! ros2 pkg list | grep -q arm_simulation; then
    echo "❌ Error: arm_simulation package not found. Please source your workspace:"
    echo "   cd ~/ros2_ws && source install/setup.bash"
    exit 1
fi

echo "✅ ROS2 and workspace are properly sourced"
echo ""

echo "Testing randomize functionality:"
echo ""
echo "Option 1: Built-in Joint State Publisher GUI Randomize"
echo "----------------------------------------"
echo "1. Start the UR5 simulation:"
echo "   ./launch_arm_sim.sh"
echo ""
echo "2. In the Joint State Publisher GUI window:"
echo "   - Click the 'Randomize' button"
echo "   - The robot should move to a random pose"
echo ""
echo "Option 2: Custom Random Pose Publisher (GUARANTEED TO WORK)"
echo "--------------------------------------------------------"
echo "1. Start the UR5 simulation in one terminal:"
echo "   ./launch_arm_sim.sh"
echo ""
echo "2. In another terminal, run the custom randomizer:"
echo "   cd ~/ros2_ws"
echo "   source install/setup.bash"
echo "   ros2 run arm_simulation random_pose_publisher.py"
echo ""
echo "The custom script will automatically move the robot to random poses every 2 seconds!"
echo ""

read -p "Would you like to test the custom randomizer now? (y/n): " choice

if [ "$choice" = "y" ] || [ "$choice" = "Y" ]; then
    echo ""
    echo "🚀 Starting UR5 simulation with custom randomizer..."
    echo ""
    echo "This will:"
    echo "1. Launch the UR5 simulation"
    echo "2. Wait 5 seconds"
    echo "3. Start the custom random pose publisher"
    echo ""
    echo "You should see the robot moving to random poses every 2 seconds!"
    echo "Press Ctrl+C to stop both processes."
    echo ""
    
    # Start the simulation in background
    cd ~/ros2_ws
    source install/setup.bash
    ./launch_arm_sim.sh &
    SIM_PID=$!
    
    echo "✅ Simulation started (PID: $SIM_PID)"
    echo "⏳ Waiting 5 seconds for simulation to initialize..."
    sleep 5
    
    echo "🎲 Starting random pose publisher..."
    ros2 run arm_simulation random_pose_publisher.py &
    RANDOM_PID=$!
    
    echo "✅ Random pose publisher started (PID: $RANDOM_PID)"
    echo ""
    echo "🎉 Both processes are running!"
    echo "Watch the robot in RViz - it should be moving to random poses!"
    echo ""
    echo "Press Ctrl+C to stop..."
    
    # Set up cleanup
    trap cleanup EXIT
    
    # Wait for user interruption
    wait
    
    cleanup() {
        echo ""
        echo "🛑 Stopping processes..."
        kill $SIM_PID 2>/dev/null
        kill $RANDOM_PID 2>/dev/null
        pkill -f "ros2\|rviz\|joint_state" 2>/dev/null
        echo "✅ All processes stopped"
        exit 0
    }
else
    echo ""
    echo "Manual testing instructions:"
    echo ""
    echo "Terminal 1:"
    echo "----------"
    echo "cd ~/ros2_ws"
    echo "./launch_arm_sim.sh"
    echo ""
    echo "Terminal 2:"
    echo "----------"
    echo "cd ~/ros2_ws"
    echo "source install/setup.bash"
    echo "ros2 run arm_simulation random_pose_publisher.py"
    echo ""
    echo "The robot will move to random poses every 2 seconds!"
fi