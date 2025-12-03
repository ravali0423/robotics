#!/bin/bash

# Speech Recognition Setup Script for ROS2 Speech-to-Gesture System
# This script installs all required dependencies for speech recognition

echo "Setting up Speech Recognition Dependencies..."

# Navigate to workspace
cd /home/ravali/ros2_ws

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Install Python dependencies in virtual environment
echo "Installing Python speech recognition libraries..."
.venv/bin/pip install SpeechRecognition sounddevice numpy

# Try to install PyAudio (may fail if system dependencies missing)
echo "Attempting to install PyAudio (optional)..."
.venv/bin/pip install PyAudio || echo "⚠️  PyAudio installation failed. Will use SoundDevice backend instead."

# Check if installation was successful
echo "Verifying installation..."
.venv/bin/python -c "
try:
    import speech_recognition as sr
    print(f'✅ SpeechRecognition: {sr.__version__}')
    
    # Check audio backends
    audio_backends = []
    try:
        import pyaudio
        audio_backends.append('PyAudio')
    except ImportError:
        pass
        
    try:
        import sounddevice as sd
        import numpy as np
        audio_backends.append(f'SoundDevice {sd.__version__}')
    except ImportError:
        pass
    
    if audio_backends:
        print(f'✅ Audio backends: {', '.join(audio_backends)}')
        print('✅ Speech recognition ready!')
    else:
        print('❌ No audio backend available')
        exit(1)
        
except ImportError as e:
    print(f'❌ Missing dependency: {e}')
    exit(1)
"

echo ""
echo "🎤 Speech Recognition Setup Complete!"
echo ""
echo "To test the system with speech recognition:"
echo "1. Run: ros2 launch arm_simulation speech_to_gesture.launch.py"
echo "2. In another terminal: ros2 topic pub /user_command std_msgs/msg/String '{data: \"start listening\"}' --once"
echo "3. Speak into your microphone to control the robotic hand"
echo ""