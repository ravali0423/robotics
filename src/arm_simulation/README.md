# Speech-to-Gesture Assistive Technology System

## Overview

This ROS2 package implements a comprehensive speech-to-gesture system designed to assist deaf and hard-of-hearing individuals by translating spoken words into American Sign Language (ASL) gestures using a robotic hand. The system serves as a bridge for communication between hearing and deaf communities.

## Features

### Core Functionality
- **Real-time speech recognition** - Converts spoken words to text
- **ASL gesture mapping** - Comprehensive library of ASL alphabet, numbers, and common words
- **Gesture sequencing** - Smooth transitions between multiple gestures for spelling and phrases
- **Audio-visual feedback** - Status indicators and audio cues for system operation
- **Interactive control** - Multiple input methods for testing and demonstration

### Assistive Technology Benefits
- **Communication bridge** - Helps hearing people communicate with deaf individuals
- **Educational tool** - Demonstrates ASL alphabet and common signs
- **Emergency assistance** - Can convey important phrases and emergency words
- **Accessibility** - Voice-controlled interface requires no manual input

## System Architecture

The system consists of several interconnected ROS2 nodes:

1. **Speech Recognition Node** (`speech_recognition_node.py`)
   - Captures audio input from microphone
   - Converts speech to text using Google Speech API or offline recognition
   - Publishes recognized text to `/recognized_speech` topic

2. **ASL Mapper** (`asl_mapper.py`)
   - Contains comprehensive ASL gesture database
   - Maps letters, numbers, and words to hand joint configurations
   - Supports finger spelling and complete word signs

3. **Enhanced Hand Controller** (`enhanced_hand_controller.py`)
   - Controls robotic hand joint positions
   - Supports complex ASL gestures beyond simple finger counting
   - Provides smooth transitions between gestures

4. **Gesture Sequencer** (`gesture_sequencer.py`)
   - Manages sequences of multiple gestures
   - Handles timing and pauses between letters/words
   - Supports playback control (play, pause, stop)

5. **Speech-to-Gesture Coordinator** (`speech_to_gesture_coordinator.py`)
   - Main orchestrator for the entire pipeline
   - Processes speech input and coordinates gesture output
   - Manages system state and user commands

6. **Audio-Visual Feedback** (`audio_visual_feedback.py`)
   - Provides audio cues for hearing users
   - Visual indicators in RViz showing system status
   - Text-to-speech announcements for important events

## Installation and Setup

### Prerequisites
- ROS2 (Jazzy or later)
- Python 3.8+
- Audio input device (microphone)
- Optional: Audio output for feedback

### Required Python packages
```bash
pip install SpeechRecognition pyaudio
```

### Build the package
```bash
cd ~/ros2_ws
colcon build --packages-select arm_simulation
source install/setup.bash
```

## Usage

### Quick Start - Complete System
Launch the full speech-to-gesture system:
```bash
ros2 launch arm_simulation speech_to_gesture.launch.py
```

This starts all components and enables voice control. The system will:
1. Start listening for speech automatically
2. Convert speech to ASL gestures
3. Display gestures on the robotic hand in RViz
4. Provide audio/visual feedback

### Testing Mode - Visual Only
For testing without audio components:
```bash
ros2 launch arm_simulation test_gestures.launch.py
```

### Manual Control Commands

#### Basic gesture commands:
```bash
# Finger counting (0-5)
ros2 topic pub /finger_count std_msgs/msg/Int32 '{data: 3}'

# Individual letters
ros2 topic pub /letter_command std_msgs/msg/String '{data: "a"}'

# Words and phrases
ros2 topic pub /gesture_command std_msgs/msg/String '{data: "hello"}'

# Text sequences (finger spelling)
ros2 topic pub /text_to_sequence std_msgs/msg/String '{data: "cat"}'
```

#### System control:
```bash
# Start/stop listening
ros2 topic pub /user_command std_msgs/msg/String '{data: "start listening"}'
ros2 topic pub /user_command std_msgs/msg/String '{data: "stop listening"}'

# Demonstrations
ros2 topic pub /user_command std_msgs/msg/String '{data: "demo alphabet"}'
ros2 topic pub /user_command std_msgs/msg/String '{data: "demo numbers"}'
ros2 topic pub /user_command std_msgs/msg/String '{data: "demo words"}'

# System status and control
ros2 topic pub /user_command std_msgs/msg/String '{data: "status"}'
ros2 topic pub /user_command std_msgs/msg/String '{data: "reset"}'
```

### Running Demonstrations

#### Comprehensive demonstration:
```bash
ros2 run arm_simulation speech_to_gesture_demo.py full
```

#### Specific demonstrations:
```bash
# ASL alphabet
ros2 run arm_simulation speech_to_gesture_demo.py alphabet

# Numbers 0-9
ros2 run arm_simulation speech_to_gesture_demo.py numbers

# Common words
ros2 run arm_simulation speech_to_gesture_demo.py words

# Interactive mode
ros2 run arm_simulation speech_to_gesture_demo.py interactive
```

### System Testing

#### Quick functionality test:
```bash
ros2 run arm_simulation system_tester.py quick
```

#### Comprehensive component testing:
```bash
ros2 run arm_simulation system_tester.py full
```

#### Specific component tests:
```bash
ros2 run arm_simulation system_tester.py fingers
ros2 run arm_simulation system_tester.py letters
ros2 run arm_simulation system_tester.py words
ros2 run arm_simulation system_tester.py sequences
```

## ASL Gesture Library

### Supported Gestures

#### Alphabet (A-Z)
Complete ASL alphabet with proper finger positioning for each letter.

#### Numbers (0-9)
Standard ASL number representations.

#### Common Words
- hello, goodbye
- please, thank you
- yes, no
- And more...

#### Emergency Phrases
- help, emergency
- call 911, doctor, hospital

### Adding Custom Gestures

You can extend the ASL library by modifying `asl_mapper.py`:

```python
# Add new word gesture
asl_mapper.save_custom_gesture('new_word', joint_configuration, 'words')
```

## Technical Details

### Joint Configuration
The robotic hand has 14 controllable joints:
- Thumb: 2 joints (base, proximal)
- Index finger: 3 joints (base, proximal, middle)
- Middle finger: 3 joints (base, proximal, middle)
- Ring finger: 3 joints (base, proximal, middle)
- Pinky finger: 3 joints (base, proximal, middle)

### Coordinate System
- Joint positions range from 0.0 (extended) to 1.4+ (fully bent)
- Base joints control finger spread/positioning
- Proximal/middle joints control finger bending

### Communication Topics

#### Published Topics:
- `/joint_states` - Hand joint positions
- `/current_gesture` - Current gesture being displayed
- `/sequence_status` - Status of gesture sequences
- `/system_status` - Overall system status
- `/system_status_markers` - Visual indicators for RViz

#### Subscribed Topics:
- `/recognized_speech` - Text from speech recognition
- `/finger_count` - Simple finger counting commands
- `/gesture_command` - Word/phrase gesture commands
- `/letter_command` - Individual letter commands
- `/text_to_sequence` - Text to convert to gesture sequence
- `/user_command` - High-level user commands

## Accessibility Features

### For Hearing Users
- Audio feedback when system is listening
- Voice confirmation of recognized speech
- Status announcements via text-to-speech

### For Deaf Users
- Visual status indicators in RViz
- Clear gesture representations
- Text display of recognized speech

### Universal Design
- Multiple input methods (voice, manual commands)
- Configurable timing and sensitivity
- Error recovery and system reset capabilities

## Troubleshooting

### Common Issues

#### No audio input detected:
- Check microphone permissions
- Verify audio device is working
- Install/configure PulseAudio if needed

#### Speech recognition not working:
- Check internet connection (for Google Speech API)
- Install offline recognition: `pip install pocketsphinx`
- Adjust microphone sensitivity in parameters

#### Gestures not displaying:
- Ensure RViz is running and configured
- Check that all nodes are publishing
- Verify joint state publisher is active

#### System not responding:
- Use reset command: `/user_command "reset"`
- Restart nodes if necessary
- Check ROS2 topic connectivity

### Debug Commands

```bash
# Check active topics
ros2 topic list

# Monitor speech recognition
ros2 topic echo /recognized_speech

# Monitor gesture status
ros2 topic echo /current_gesture

# Monitor system status
ros2 topic echo /system_status
```

## Future Enhancements

### Planned Features
- Support for more sign languages (BSL, FSL, etc.)
- Machine learning for gesture recognition
- Mobile app interface
- Multi-hand coordination for complex signs
- Real-time conversation mode

### Hardware Extensions
- Integration with physical robotic hands
- Haptic feedback systems
- Eye tracking for gaze-based control
- Integration with smart home systems

## Contributing

This project welcomes contributions to improve accessibility and expand ASL support. Areas for contribution:
- Additional ASL gestures and phrases
- Support for other sign languages
- Improved speech recognition accuracy
- Better gesture timing and transitions
- Hardware integration

## License

This project is licensed under the Apache-2.0 License - see the LICENSE file for details.

## Acknowledgments

This assistive technology project is dedicated to supporting the deaf and hard-of-hearing community. We acknowledge the importance of American Sign Language as a rich, complete language and aim to promote understanding and communication accessibility.

---

**Important Note**: This system is designed as an assistive tool and educational resource. It should not replace learning proper ASL from qualified instructors or interaction with the deaf community. We encourage users to learn about deaf culture and proper ASL communication practices.