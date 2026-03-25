# Humanoid Upper Body Model — Real-World ASL Display

## Overview

A new realistic humanoid upper body model has been added to the robotics project. This model resembles a proper human upper body with face, head, neck, shoulders, arms, and hands, providing a natural context for demonstrating American Sign Language (ASL) gestures.

## Model Components

### Head and Face

- **Head**: Spherical shape with realistic proportions (15cm radius)
- **Face**: Front plate with skin tone
- **Eyes**: Two dark spheres positioned naturally
- **Mouth**: Red colored mouth element
- **Facial expressions**: Provides communication context

### Neck and Shoulders

- **Neck**: Cylindrical connector (12cm long) links head to torso
- **Shoulders**: Wide shoulder bar connects both arms
- Natural shoulder width positioning

### Torso

- **Clothing**: Dark gray shirt
- **Chest**: Skin tone at neckline for realism
- **Stability**: Provides stable base for gestures

### Arms

- **Upper arm**: 35cm cylinder with natural skin tone
- **Forearm**: 30cm cylinder for extended reach
- **Wrist**: 8cm segment connecting to hand
- **Structure**: Realistic proportions for human arm

### Hands

- **Palm**: 11cm × 8.2cm base for gesture platform
- **Fingers**: Full articulation with:
  - Thumb: 2 joints
  - Index, Middle, Ring, Pinky: 3 joints each
- **Skin tone**: Matches body for consistency
- **Nail details**: For realistic appearance

## Files Created

### URDF Models

- `src/arm_simulation/urdf/humanoid_upper_body.urdf.xacro` - Basic humanoid body
- `src/arm_simulation/urdf/humanoid_full_body.urdf.xacro` - Full humanoid with hand articulation

### Launch Files

- `src/arm_simulation/launch/humanoid_upper_body.launch.py` - Standard launch configuration

### RViz Configuration

- `src/arm_simulation/rviz/humanoid_upper_body.rviz` - Visualization setup optimized for humanoid

## Launching the Model

### Inside Docker Container

```bash
# Basic launch
ros2 launch arm_simulation humanoid_upper_body.launch.py
```

### Full Launch from Mac

```bash
# Terminal 1: Start container
cd /path/to/robotics
docker run -it --name arm-sim -p 5900:5900 arm-sim

# Terminal 1 (inside container): Launch the humanoid
ros2 launch arm_simulation humanoid_upper_body.launch.py

# Terminal 2 (Mac): Open VNC Viewer
open -a "VNC Viewer"
# Connect to: localhost:5900

# Terminal 3 (new docker exec): Send commands
docker exec -it arm-sim bash
```

## Control Commands

### Two-Character Words (Dual Hand)

Right hand displays 1st character, left hand displays 2nd character:

```bash
# Examples
ros2 topic pub /letter_command std_msgs/msg/String '{data: "ab"}' --once
ros2 topic pub /letter_command std_msgs/msg/String '{data: "go"}' --once
ros2 topic pub /letter_command std_msgs/msg/String '{data: "hi"}' --once
ros2 topic pub /letter_command std_msgs/msg/String '{data: "on"}' --once
ros2 topic pub /letter_command std_msgs/msg/String '{data: "it"}' --once
```

### Single Letters (Both Hands)

```bash
ros2 topic pub /letter_command std_msgs/msg/String '{data: "a"}' --once
ros2 topic pub /letter_command std_msgs/msg/String '{data: "z"}' --once
```

### Finger Counting (0–10)

Splits automatically: left=min(count,5), right=max(0,count-5)

```bash
ros2 topic pub /finger_count std_msgs/msg/Int32 '{data: 3}' --once
ros2 topic pub /finger_count std_msgs/msg/Int32 '{data: 8}' --once
ros2 topic pub /finger_count std_msgs/msg/Int32 '{data: 10}' --once
```

### Independent Hand Control

```bash
# Left hand letter
ros2 topic pub /left/letter_command std_msgs/msg/String '{data: "l"}' --once

# Right hand letter
ros2 topic pub /right/letter_command std_msgs/msg/String '{data: "r"}' --once

# Finger count per hand
ros2 topic pub /left/finger_count std_msgs/msg/Int32 '{data: 3}' --once
ros2 topic pub /right/finger_count std_msgs/msg/Int32 '{data: 5}' --once

# Gesture per hand
ros2 topic pub /left/gesture_command std_msgs/msg/String '{data: "v"}' --once
ros2 topic pub /right/gesture_command std_msgs/msg/String '{data: "a"}' --once
```

## Use Cases

### 1. Educational Demonstrations

- Teach ASL alphabet and numbers
- Show communication with deaf community
- Real-world context for learning

### 2. Communication Assistance

- Help hearing people understand sign language
- Bridge communication gaps
- Assistive technology for interaction

### 3. Interactive Learning

- Face expressions add communication context
- Natural gesture positioning
- Realistic human-like presentation

### 4. Public Demonstrations

- More impressive than abstract hand models
- Shows respect for deaf community
- Professional appearance

### 5. Emergency Services

- Display emergency phrases naturally
- Critical communication aid
- Realistic medical context

## Model Dimensions

| Component | Dimension                       |
| --------- | ------------------------------- |
| Head      | 30cm diameter                   |
| Neck      | 12cm long, 8cm diameter         |
| Shoulders | 45cm wide, 8cm high             |
| Torso     | 40cm wide, 45cm tall, 20cm deep |
| Upper Arm | 35cm long, 10cm diameter        |
| Forearm   | 30cm long, 7.6cm diameter       |
| Wrist     | 8cm long, 7cm diameter          |
| Hand Palm | 11cm × 8.2cm × 1.8cm            |

## Materials and Colors

| Part           | Color       | RGB              |
| -------------- | ----------- | ---------------- |
| Face           | Light Skin  | 0.96, 0.78, 0.62 |
| Body/Arms      | Medium Skin | 0.95, 0.76, 0.60 |
| Hands          | Light Skin  | 0.96, 0.78, 0.62 |
| Eyes           | Dark        | 0.2, 0.2, 0.2    |
| Mouth          | Red         | 0.8, 0.3, 0.3    |
| Torso Clothing | Dark Gray   | 0.2, 0.2, 0.2    |

## Customization

### Changing Colors

Edit `humanoid_upper_body.urdf.xacro`:

```xml
<material name="skin_face">
  <color rgba="0.96 0.78 0.62 1.0"/>  <!-- RGBA values -->
</material>
```

### Adjusting Proportions

Modify xacro properties at the top of the file:

```xml
<xacro:property name="head_radius" value="0.15" />  <!-- 15cm head -->
<xacro:property name="torso_height" value="0.45" />  <!-- 45cm torso -->
```

### Rebuilding Docker Image

After changes:

```bash
docker rm -f arm-sim
docker build -t arm-sim .
docker run -it --name arm-sim -p 5900:5900 arm-sim
```

## Advantages Over Abstract Models

### Humanoid Body (New)

✅ **Realistic appearance** - Looks like human upper body
✅ **Better context** - Face provides communication cues
✅ **Professional** - Impressive for demonstrations
✅ **Educational** - Teaches respect for deaf culture
✅ **Accessible** - Easier for hearing people to understand

### Dual-Hand Only (Previous)

⚠️ No body context
⚠️ Abstract presentation
⚠️ Less educational value
✅ Still useful for technical testing

## Integration with Existing System

The humanoid model fully integrates with existing components:

- ✅ Speech-to-gesture coordinator
- ✅ Hand controllers (left/right)
- ✅ Gesture sequencer
- ✅ ASL mapper
- ✅ Audio-visual feedback
- ✅ All existing topics and commands

## Performance Notes

- **Model complexity**: Advanced (head + shoulders + hands with fingers)
- **computation**: Suitable for modern computers
- **VNC performance**: Smooth over network with ~1.5MB bandwidth
- **Update rate**: 30Hz in RViz

## Troubleshooting

### Hands not showing fingers

- Ensure `humanoid_full_body.urdf.xacro` is being used
- Check hand controller nodes are running
- Verify topic publishing: `ros2 topic list | grep letter_command`

### Model appears distorted in RViz

- Close and reopen RViz window
- Reset camera view with "Frame" → "Target Frame" → "world"
- Verify robot_description is published: `ros2 param get /robot_state_publisher robot_description`

### Gestures not appearing

- Confirm hand controller is running: `ros2 node list`
- Check topic is being published: `ros2 topic echo /letter_command`
- Verify coordinate frames: `ros2 run tf2_tools view_frames`

## Future Enhancements

Potential improvements:

- [ ] Eye gaze control for emphasis
- [ ] Head rotation for natural gestures
- [ ] Body posture adjustments
- [ ] Additional facial expressions
- [ ] Hair detail
- [ ] Clothing texturing

## References

- ROS2 URDF Documentation: http://wiki.ros.org/urdf
- Xacro Macros: http://wiki.ros.org/xacro
- RViz Visualization: http://wiki.ros.org/rviz
- ASL Gesture Standards: https://www.lifeprint.com/

## Support

For issues or improvements, consider:

1. Check console output in launch terminal
2. Review ROS2 logs: `ros2 topic echo /rosout`
3. Verify docker container is running: `docker ps`
4. Check VNC connection: `netstat -an | grep 5900`
