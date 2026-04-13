# Robotics Simulation Workspace

A ROS 2 Jazzy workspace with two packages:

- **`arm_simulation`** — Robotic hand simulations with ASL gesture, speech-to-gesture, and full humanoid sign language interpreter
- **`warehouse_robot`** — Autonomous warehouse delivery robot with web joystick control

---

## Table of Contents

1. [Project Structure](#project-structure)
2. [Prerequisites](#prerequisites)
3. [Docker Setup (Mac & Windows)](#docker-setup-mac--windows)
4. [Native Setup (Linux)](#native-setup-linux)
5. [Arm Simulation](#arm-simulation)
   - [Robotic Hand — Single Hand](#robotic-hand--single-hand)
   - [Dual Hands](#dual-hands)
   - [Humanoid Sign Language Interpreter](#humanoid-sign-language-interpreter)
   - [Speech-to-Gesture System](#speech-to-gesture-system)
6. [Warehouse Robot](#warehouse-robot)
7. [ROS Topics Reference](#ros-topics-reference)
8. [Troubleshooting](#troubleshooting)

---

## Project Structure

```
robotics/
├── Dockerfile
├── README.md
└── src/
    ├── arm_simulation/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   ├── robotic_hand.launch.py        # Single robotic hand
    │   │   ├── dual_hands.launch.py          # Two hands side by side
    │   │   ├── humanoid_robot.launch.py      # Full humanoid sign language interpreter
    │   │   └── speech_to_gesture.launch.py   # Full speech-to-gesture pipeline
    │   ├── urdf/
    │   │   ├── robotic_hand.urdf.xacro       # Single hand model
    │   │   ├── dual_robotic_hands.urdf.xacro # Dual hand model
    │   │   └── humanoid_robot.urdf.xacro     # Full humanoid body (head to toe)
    │   ├── rviz/
    │   │   ├── robotic_hand.rviz
    │   │   ├── dual_hands.rviz
    │   │   ├── humanoid_robot.rviz           # Full-body view
    │   │   └── speech_to_gesture.rviz
    │   └── scripts/
    │       ├── hand_controller.py
    │       ├── enhanced_hand_controller.py
    │       ├── dual_hand_coordinator.py
    │       ├── asl_mapper.py                 # ASL hand shape library
    │       ├── asl_body_postures.py          # ASL body posture data (hello, 1–10, dance moves)
    │       ├── body_controller.py            # Humanoid body joint controller
    │       ├── humanoid_sign_coordinator.py  # Sign language orchestrator
    │       ├── gesture_sequencer.py
    │       ├── speech_recognition_node.py
    │       ├── speech_to_gesture_coordinator.py
    │       ├── audio_visual_feedback.py
    │       └── setup_speech_recognition.sh
    └── warehouse_robot/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        ├── urdf/
        ├── worlds/
        └── scripts/
            ├── web_joystick_controller.py    # Primary: web-based joystick UI
            ├── manual_robot_controller.py    # Backup: terminal WASD control
            ├── robot_controller.py           # Autonomous waypoint navigation
            ├── car_controller.py             # Low-level vehicle control
            ├── generate_waypoints.py
            ├── generate_new_waypoints.py
            ├── show_waypoints.py
            └── mission_log_manager.py
```

---

## Prerequisites

### System Requirements

- **OS**: Ubuntu 24.04 LTS (native) or Docker (Mac/Windows)
- **ROS 2**: Jazzy
- **Build tools**: `colcon`, `rosdep`
- **Python**: 3.10+

### Required ROS 2 Packages

```bash
sudo apt update
sudo apt install \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro
```

### Optional — Speech Recognition

```bash
pip install SpeechRecognition pyaudio
sudo apt install portaudio19-dev python3-pyaudio
```

> Without audio hardware, the speech-to-gesture system runs in text-only mode — all gesture control still works via ROS topics.

---

## Docker Setup (Mac & Windows)

The Docker image bundles ROS 2 Jazzy, all dependencies, and a VNC server so you can view RViz2 from Mac or Windows without a native Linux install.

### Step 1 — Install Docker Desktop

Download and install [Docker Desktop](https://www.docker.com/products/docker-desktop/). Ensure it is running before proceeding.

### Step 2 — Install VNC Viewer

**Mac:**
```bash
brew install --cask vnc-viewer
```

**Windows (PowerShell):**
```powershell
winget install --id RealVNC.VNCViewer -e
```

### Step 3 — Build the Docker Image

```bash
cd /path/to/robotics
docker build -t arm-sim .
```

The first build takes several minutes. Subsequent builds use the layer cache.

> You may see a platform warning (`linux/amd64` on `linux/arm64`) — this is harmless. The container runs via Rosetta 2 emulation on Apple Silicon.

### Step 4 — Run the Container

**Mac:**
```bash
docker run -it \
  --name arm-sim \
  -p 5900:5900 \
  arm-sim
```

**Windows:**
```cmd
docker run -it --name arm-sim -p 5900:5900 --security-opt seccomp=unconfined arm-sim
```

You will see:
```
The VNC desktop is: 0.0.0.0:0
PORT=5900
root@<container-id>:/ros2_ws#
```

You are now inside the container shell.

### Step 5 — Connect via VNC

**Mac** — open a new terminal tab and run:
```bash
open -a "VNC Viewer"
```
Connect to `localhost:5900`. Leave the password blank and click **Continue**.

**Windows** — open a new PowerShell window and run:
```powershell
& "C:\Program Files\RealVNC\VNC Viewer\vncviewer.exe" localhost:5900
```

> Use the **Scale to fit** button in VNC Viewer to fit the desktop to your window.

### Step 6 — Launch a Simulation

In the **container terminal**, run one of the launch commands from the sections below. The RViz2 window will appear in the VNC Viewer.

### Container Management

| Task | Command |
|------|---------|
| Stop the container | `docker stop arm-sim` |
| Restart and re-attach | `docker start -ai arm-sim` |
| Open a second terminal in the container | `docker exec -it arm-sim bash` |
| Remove the container (keeps image) | `docker rm arm-sim` |
| Rebuild after code changes | `docker rm -f arm-sim && docker build -t arm-sim . && docker run -it --name arm-sim -p 5900:5900 arm-sim` |

---

## Native Setup (Linux)

```bash
# Clone the workspace
git clone <repository-url> ~/ros2_ws
cd ~/ros2_ws

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

---

## Arm Simulation

### Robotic Hand — Single Hand

A 5-finger robotic hand with ASL gesture support and real-time finger counting.

**Launch:**
```bash
ros2 launch arm_simulation robotic_hand.launch.py
```

**Open a second terminal into the container (Docker) or a new terminal (native):**
```bash
docker exec -it arm-sim bash   # Docker only
source install/setup.bash
```

**Finger counting (0–5):**
```bash
# Fist
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 0}" --once

# 1 finger
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 1}" --once

# 3 fingers
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 3}" --once

# Open hand (5 fingers)
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 5}" --once
```

**ASL letter gestures:**
```bash
# Letter A (fist)
ros2 topic pub /gesture_command std_msgs/msg/String "{data: a}" --once

# Letter L shape
ros2 topic pub /gesture_command std_msgs/msg/String "{data: l}" --once

# Letter V (peace sign)
ros2 topic pub /gesture_command std_msgs/msg/String "{data: v}" --once

# Letter I (pinky up)
ros2 topic pub /gesture_command std_msgs/msg/String "{data: i}" --once

# Reset to neutral
ros2 topic pub /gesture_command std_msgs/msg/String "{data: neutral}" --once
```

Supported gestures: `a` through `z`, `hello`, `neutral`

**Spell a letter via letter_command:**
```bash
ros2 topic pub /letter_command std_msgs/msg/String "{data: r}" --once
```

---

### Dual Hands

Two robotic hands side by side. Supports combined finger counting (0–10), two-character word spelling, single-letter display on both hands, and independent per-hand control.

**Launch:**
```bash
ros2 launch arm_simulation dual_hands.launch.py
```

Open a second terminal into the container or a new native terminal, then:

**Combined finger count (0–10) — splits automatically across both hands:**
```bash
# 8 total → left=5, right=3
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 8}" --once

# 3 total → left=3, right=0
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 3}" --once

# 10 total → left=5, right=5
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 10}" --once
```

**Two-character word spelling — right hand = first character, left hand = second:**
```bash
ros2 topic pub /letter_command std_msgs/msg/String "{data: ab}" --once
ros2 topic pub /letter_command std_msgs/msg/String "{data: go}" --once
ros2 topic pub /letter_command std_msgs/msg/String "{data: it}" --once
```

**Single letter — both hands display the same letter:**
```bash
ros2 topic pub /letter_command std_msgs/msg/String "{data: a}" --once
ros2 topic pub /letter_command std_msgs/msg/String "{data: z}" --once
```

**Independent hand control:**
```bash
# Per-hand gestures
ros2 topic pub /left/gesture_command std_msgs/msg/String "{data: v}" --once
ros2 topic pub /right/gesture_command std_msgs/msg/String "{data: a}" --once

# Per-hand ASL letters
ros2 topic pub /left/letter_command std_msgs/msg/String "{data: l}" --once
ros2 topic pub /right/letter_command std_msgs/msg/String "{data: r}" --once

# Per-hand finger count
ros2 topic pub /left/finger_count std_msgs/msg/Int32 "{data: 3}" --once
ros2 topic pub /right/finger_count std_msgs/msg/Int32 "{data: 5}" --once
```

---

### Humanoid Sign Language Interpreter

A full humanoid robot (~1.5 m tall) that performs American Sign Language signs with coordinated arm, hand, and head motion. Send a single `/sign_command` topic and the entire body animates: arms raise to the correct position, fingers form the right shape, and the head follows — all simultaneously.

#### System Architecture

| Node | Script | Role |
|------|--------|------|
| Sign Coordinator | `humanoid_sign_coordinator.py` | Receives `/sign_command`, sequences body + hand commands in a background thread |
| Body Controller | `body_controller.py` | Drives 10 body joints (shoulders, elbows, wrists, neck) at 10 Hz with smooth interpolation |
| Left Hand Controller | `enhanced_hand_controller.py` | Controls 14 left finger joints |
| Right Hand Controller | `enhanced_hand_controller.py` | Controls 14 right finger joints |
| Dual Hand Coordinator | `dual_hand_coordinator.py` | Routes combined hand commands to left/right controllers |
| Robot State Publisher | (ROS built-in) | Merges all `/joint_states` and publishes TF for RViz |

#### Launch

```bash
ros2 launch arm_simulation humanoid_robot.launch.py
```

The RViz window opens with a full-body view. The robot starts in a neutral standing pose.

#### Performing Signs

Open a second terminal in the container:
```bash
docker exec -it arm-sim bash
```

**Hello** — right arm raises to forehead and sweeps outward:
```bash
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "hello"'
```

**Numbers 1–9** — both arms move to a signing-ready position, right hand forms the number shape:
```bash
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "1"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "5"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "9"'
```

**Number 10** — right wrist oscillates (ASL thumb wave):
```bash
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "10"'
```

**Common interaction commands** — greetings, social responses, and directives:

| Command | Description |
|---------|-------------|
| `bye` | Right arm raised, hand waves side to side three times |
| `welcome` | Both arms spread wide with a slight forward bow |
| `please` | Flat hand makes two circles over the chest |
| `thanks` | Flat hand near chin sweeps forward and down |
| `yes` | Head nods twice |
| `no` | Head shakes twice |
| `sorry` | Closed fist makes two circles over the chest |
| `come` | Arm extends forward then beckons twice |
| `stop` | Right arm raised with palm facing out — halt signal |
| `good` | Flat hand near chin sweeps forward and down |

```bash
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "bye"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "welcome"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "please"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "thanks"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "yes"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "no"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "sorry"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "come"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "stop"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "good"'
```

**Hello** — right arm raises to forehead and sweeps outward (already shown above)

**Why** — alternating arm swing with head tilts (repeated rhythm):
```bash
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "why"'
```

**Alphabet A–Z** — right arm moves to signing position, right hand forms the ASL letter shape:
```bash
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "a"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "b"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "z"'
```

**Two-letter words / simultaneous two-hand signs** — both hands sign at the same time:
```bash
# Two-letter word (left hand = first letter, right hand = second)
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "hi"'

# Or space-separated tokens
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "a b"'
```

**Supported signs:** `hello`, `why`, `a`–`z`, `1`–`10`

**Supported interaction commands:** `bye`, `welcome`, `please`, `thanks`, `yes`, `no`, `sorry`, `come`, `stop`, `good`

**Sequence commands** — run a full set in one shot:

| Command | What it does |
|---------|-------------|
| `numbers` | Signs 1 through 10 one by one |
| `alphabet` | Signs a through z one by one |
| `dance_all` | Runs all 5 dance moves back to back |

```bash
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "numbers"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "alphabet"'
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "dance_all"'
```

**Dance moves** — full-body choreographed sequences:
```bash
# Stiff mechanical arm snaps with head turns
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "robot"'

# Both arms shoot up and wave, jazz hands finish
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "celebrate"'

# Alternating arm raises — friendly crowd wave
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "wave"'

# Funky alternating arm sweeps + double pump
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "groove"'

# Classic Saturday-Night-Fever alternating diagonal point
ros2 topic pub -1 /sign_command std_msgs/msg/String 'data: "disco"'
```

**Supported dance commands:** `robot`, `celebrate`, `wave`, `groove`, `disco`

#### Manual Body Control (Testing)

Drive individual body joints directly without a sign command:
```bash
# Raise right arm
ros2 topic pub -1 /body_pose_command std_msgs/msg/String \
  'data: "{\"right_shoulder_pitch\": 0.8, \"right_elbow_pitch\": 1.2}"'

# Return to neutral
ros2 topic pub -1 /body_pose_command std_msgs/msg/String 'data: "neutral"'

# Signing-ready pose
ros2 topic pub -1 /body_pose_command std_msgs/msg/String 'data: "signing_ready"'
```

#### Rebuild After Code Changes

The new files are baked into the Docker image at build time. After any change to scripts or URDF:
```bash
# On your host machine, from the robotics/ directory
docker rm -f arm-sim
docker build -t arm-sim .
docker run -it --name arm-sim -p 5900:5900 arm-sim
```

---

### Speech-to-Gesture System

Translates spoken words into ASL gestures in real time. The system runs fully in text-only mode if no microphone is available.

#### System Architecture

| Node | Script | Role |
|------|--------|------|
| Speech Recognition | `speech_recognition_node.py` | Captures audio, converts to text via Google Speech API or offline engine, publishes to `/recognized_speech` |
| ASL Mapper | `asl_mapper.py` | Maps letters, numbers, and words to hand joint configurations |
| Enhanced Hand Controller | `enhanced_hand_controller.py` | Controls hand joint positions with smooth transitions |
| Gesture Sequencer | `gesture_sequencer.py` | Manages multi-gesture sequences with timing and pause/stop support |
| Speech-to-Gesture Coordinator | `speech_to_gesture_coordinator.py` | Main pipeline orchestrator |
| Audio-Visual Feedback | `audio_visual_feedback.py` | Audio cues and RViz status markers |

#### Launch

```bash
ros2 launch arm_simulation speech_to_gesture.launch.py
```

This starts all nodes. The system begins listening for speech automatically and displays gestures on the robotic hand in RViz.

#### Text-Only Control (No Microphone Required)

All gesture functionality works via ROS topics without audio:

```bash
# Convert text to gesture sequence (finger spelling)
ros2 topic pub /text_to_sequence std_msgs/msg/String '{data: "cat"}' --once

# Individual letters
ros2 topic pub /letter_command std_msgs/msg/String '{data: "a"}' --once

# Pre-defined word gestures
ros2 topic pub /gesture_command std_msgs/msg/String '{data: "hello"}' --once

# Finger counting
ros2 topic pub /finger_count std_msgs/msg/Int32 '{data: 3}' --once
```

#### High-Level User Commands

```bash
# Start / stop voice listening
ros2 topic pub /user_command std_msgs/msg/String '{data: "start listening"}' --once
ros2 topic pub /user_command std_msgs/msg/String '{data: "stop listening"}' --once

# Run built-in demonstrations
ros2 topic pub /user_command std_msgs/msg/String '{data: "demo alphabet"}' --once
ros2 topic pub /user_command std_msgs/msg/String '{data: "demo numbers"}' --once
ros2 topic pub /user_command std_msgs/msg/String '{data: "demo words"}' --once

# Inline gesture from text
ros2 topic pub /user_command std_msgs/msg/String '{data: "gesture hello"}' --once

# Status and reset
ros2 topic pub /user_command std_msgs/msg/String '{data: "status"}' --once
ros2 topic pub /user_command std_msgs/msg/String '{data: "reset"}' --once
```

#### ASL Gesture Library

- **Alphabet**: A–Z complete ASL hand shapes
- **Numbers**: 0–9 standard ASL representations
- **Common words**: hello, goodbye, please, thank you, yes, no
- **Emergency phrases**: help, emergency, call 911, doctor, hospital

**Adding custom gestures** — edit `asl_mapper.py`:
```python
asl_mapper.save_custom_gesture('new_word', joint_configuration, 'words')
```

#### Hand Joint Configuration

The robotic hand has 14 controllable joints:

| Finger | Joints |
|--------|--------|
| Thumb | base, proximal (2) |
| Index | base, proximal, middle (3) |
| Middle | base, proximal, middle (3) |
| Ring | base, proximal, middle (3) |
| Pinky | base, proximal, middle (3) |

Joint positions range from `0.0` (fully extended) to `1.4+` (fully bent).

#### Enabling Speech Recognition

To enable voice input on a system with audio hardware:

```bash
# Install audio dependencies
sudo apt install portaudio19-dev python3-pyaudio
pip install SpeechRecognition pyaudio

# Run the setup script
./src/arm_simulation/scripts/setup_speech_recognition.sh
```

**Verify your environment:**
```bash
# Check for audio devices
ls /dev/snd/ 2>/dev/null || echo "No audio devices found"

# Check if running in Docker
[ -f /.dockerenv ] && echo "Running in Docker" || echo "Running natively"

# Test speech recognition library
python3 -c "import speech_recognition as sr; print('SR available'); mic = sr.Microphone(); print('Microphone available')"
```

**Docker with audio support:**
```bash
docker run -it --privileged \
  --device /dev/snd \
  -v /dev/shm:/dev/shm \
  -p 5900:5900 \
  arm-sim
```

---

## Warehouse Robot

An autonomous warehouse delivery robot with web joystick control, waypoint navigation, and mission logging.

### Launch

```bash
# Terminal 1 — start the Gazebo simulation
ros2 launch warehouse_robot warehouse_with_joystick.launch.py

# Terminal 2 — start the web joystick controller
cd ~/ros2_ws
python3 src/warehouse_robot/scripts/web_joystick_controller.py
```

Open `http://localhost:8080/joystick.html` in a browser to use the drag-drop joystick UI.

### Manual Terminal Control (Backup)

```bash
python3 src/warehouse_robot/scripts/manual_robot_controller.py
```

Uses WASD keys with real-time speed adjustment.

### Direct cmd_vel Control

```bash
# Move forward
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" --once

# Turn left
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}" --once

# Turn right
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{angular: {z: -1.0}}" --once

# Stop
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{}" --once
```

### Autonomous Navigation

```bash
source install/setup.bash

# Go to package pickup location
ros2 run warehouse_robot robot_controller.py pickup

# Deliver package to destination
ros2 run warehouse_robot robot_controller.py deliver

# Return to start
ros2 run warehouse_robot robot_controller.py return

# Full mission: pickup → deliver → return (with logging)
ros2 run warehouse_robot robot_controller.py mission

# Move to specific coordinates
ros2 run warehouse_robot robot_controller.py goto 2.0 3.0

# Show robot status
ros2 run warehouse_robot robot_controller.py status

# Stop immediately
ros2 run warehouse_robot robot_controller.py stop
```

### Waypoint Management

```bash
# Generate new waypoints interactively
python3 src/warehouse_robot/scripts/generate_new_waypoints.py

# View current waypoints
python3 src/warehouse_robot/scripts/show_waypoints.py
```

### Mission Logging

Mission logs are saved to `~/ros2_ws/mission_logs/` and include timestamps, positions, commands, and duration.

```bash
# List all mission logs
python3 src/warehouse_robot/scripts/mission_log_manager.py list

# View the latest log
python3 src/warehouse_robot/scripts/mission_log_manager.py latest

# Summary across all missions
python3 src/warehouse_robot/scripts/mission_log_manager.py summary

# Clean old logs (keep 10 newest)
python3 src/warehouse_robot/scripts/mission_log_manager.py clean

# Keep only 5 newest
python3 src/warehouse_robot/scripts/mission_log_manager.py clean 5
```

### Keyboard Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/model/warehouse_car/cmd_vel
```

---

## ROS Topics Reference

### Arm Simulation — Single Hand

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/finger_count` | `std_msgs/Int32` | Input | Set finger count 0–5 |
| `/gesture_command` | `std_msgs/String` | Input | Named gesture (a–z, hello, neutral) |
| `/letter_command` | `std_msgs/String` | Input | Single ASL letter (a–z) |
| `/text_to_sequence` | `std_msgs/String` | Input | Spell out any text as a gesture sequence |
| `/user_command` | `std_msgs/String` | Input | High-level commands (demo, reset, status…) |
| `/recognized_speech` | `std_msgs/String` | Input | Text from speech recognition node |
| `/joint_states` | `sensor_msgs/JointState` | Output | Current hand joint positions |
| `/current_gesture` | `std_msgs/String` | Output | Active gesture name |
| `/sequence_status` | `std_msgs/String` | Output | Gesture sequence playback status |
| `/system_status` | `std_msgs/String` | Output | Overall system status |

### Arm Simulation — Humanoid Sign Language Interpreter

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/sign_command` | `std_msgs/String` | Input | Trigger a full sign: `hello`, `1`–`10` |
| `/body_pose_command` | `std_msgs/String` | Input | Pose name (`neutral`, `signing_ready`) or JSON joint dict |
| `/right/gesture_command` | `std_msgs/String` | Input | Right hand shape (routed from sign coordinator) |
| `/left/gesture_command` | `std_msgs/String` | Input | Left hand shape |
| `/sign_status` | `std_msgs/String` | Output | `performing` \| `complete` \| `busy` \| `error` |
| `/body_status` | `std_msgs/String` | Output | `ready` \| `moving` \| `<pose_name>` |
| `/joint_states` | `sensor_msgs/JointState` | Output | All 38 joints (10 body + 14×2 hands) merged |

### Arm Simulation — Dual Hands (additional topics)

| Topic | Type | Description |
|-------|------|-------------|
| `/finger_count` | `std_msgs/Int32` | Combined count 0–10, splits automatically |
| `/left/finger_count` | `std_msgs/Int32` | Left hand only (0–5) |
| `/right/finger_count` | `std_msgs/Int32` | Right hand only (0–5) |
| `/left/gesture_command` | `std_msgs/String` | Gesture on left hand |
| `/right/gesture_command` | `std_msgs/String` | Gesture on right hand |
| `/left/letter_command` | `std_msgs/String` | ASL letter on left hand |
| `/right/letter_command` | `std_msgs/String` | ASL letter on right hand |

### Useful Diagnostics

```bash
# List all active topics
ros2 topic list

# List all running nodes
ros2 node list

# Monitor joint states
ros2 topic echo /joint_states

# Watch the active gesture
ros2 topic echo /current_gesture

# Watch speech recognition output
ros2 topic echo /recognized_speech

# Watch system status
ros2 topic echo /system_status
```

---

## Troubleshooting

### Arm Simulation

**Package not found after build:**
```bash
cd ~/ros2_ws
colcon build --packages-select arm_simulation
source install/setup.bash
```

**RViz shows no robot:**
```bash
# Verify robot_description is being published
ros2 topic echo /robot_description --once

# Check robot_state_publisher is running
ros2 node list | grep robot_state_publisher
```

**Hand does not respond to commands:**
```bash
# Check that hand_controller is running
ros2 node list | grep hand_controller

# Confirm the topic exists
ros2 topic info /finger_count

# View node subscriptions
ros2 node info /hand_controller
```

**Humanoid robot sign command ignored ("busy"):**
- A sign is already in progress. Wait for `/sign_status` to publish `complete`, then send the next command.

**Body joints not moving after `/body_pose_command`:**
```bash
# Confirm body_controller is running
ros2 node list | grep body_controller

# Check it is receiving commands
ros2 topic echo /body_pose_command
```

**RViz shows hands detached from arms:**
- The URDF was not rebuilt. Rebuild the Docker image so the new `humanoid_robot.urdf.xacro` is picked up.

**Warning: "KDL does not support root link with inertia"**
- Harmless — the simulation works correctly.

**Speech recognition not working:**
```bash
# Test library availability
python3 -c "import speech_recognition as sr; sr.Microphone()"

# Install offline fallback
pip install pocketsphinx

# Use text-only mode instead
ros2 topic pub /text_to_sequence std_msgs/msg/String '{data: "hello"}' --once
```

### Docker / VNC

**VNC Viewer stuck on connecting:**
- macOS built-in Screen Sharing does not work — use VNC Viewer only.

**VNC connection failed or cannot connect to display:**
```bash
docker rm -f arm-sim
docker run -it --name arm-sim -p 5900:5900 arm-sim
```
Connect with VNC Viewer to `localhost:5900` before running any launch commands.

**Blank screen in VNC:**
- No simulation has been launched yet — run a launch command in the container terminal.

**Cannot move the RViz window:**
- The Openbox window manager is included — windows have title bars and can be dragged.

**Build fails with `python3-pyaudio` error:**
- Already handled — the Dockerfile skips it with `--skip-keys`.

**Platform warning during build (`linux/amd64` on `linux/arm64`):**
- Harmless — the image runs via Rosetta 2 on Apple Silicon.
