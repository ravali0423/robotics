#!/bin/bash

# Speech-to-Gesture Demo Script (Text-Only Mode)
# This script demonstrates the full functionality without requiring audio hardware

echo "🤖 Starting Speech-to-Gesture System Demo (Text-Only Mode)"
echo "=============================================================="
echo ""

# Check if system is running
if ! pgrep -f "speech_to_gesture.launch.py" > /dev/null; then
    echo "⚠️  System not running. Please start it first:"
    echo "   ros2 launch arm_simulation speech_to_gesture.launch.py"
    echo ""
    echo "Then run this demo script in another terminal."
    exit 1
fi

echo "✅ System detected running. Starting demonstration..."
echo ""

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Demo sequence
DEMO_COMMANDS=(
    "demo alphabet"
    "demo numbers" 
    "demo words"
    "gesture hello"
    "gesture thank you"
    "gesture help"
)

echo "🎯 Running demonstration sequence:"
echo ""

for i in "${!DEMO_COMMANDS[@]}"; do
    cmd="${DEMO_COMMANDS[$i]}"
    echo "[$((i+1))/${#DEMO_COMMANDS[@]}] Executing: '$cmd'"
    
    # Send the command
    ros2 topic pub /user_command std_msgs/msg/String "{data: \"$cmd\"}" --once
    
    # Wait between commands
    sleep 4
    echo ""
done

echo "✅ Demo complete!"
echo ""
echo "🎯 Available Commands for Manual Testing:"
echo "   ros2 topic pub /user_command std_msgs/msg/String '{data: \"gesture hello\"}' --once"
echo "   ros2 topic pub /letter_command std_msgs/msg/String '{data: \"a\"}' --once"
echo "   ros2 topic pub /finger_count std_msgs/msg/Int32 '{data: 3}' --once"
echo "   ros2 topic pub /text_to_sequence std_msgs/msg/String '{data: \"help\"}' --once"
echo ""
echo "📖 See SPEECH_RECOGNITION_SETUP.md for more details"