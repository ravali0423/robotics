# Arm Simulation — Docker Setup on Mac (Apple Silicon)

ROS 2 Jazzy robotic arm simulation running in Docker, visualized via VNC on Mac.

Two simulations included:

- **UR5 Industrial Arm** — 6-DOF arm with interactive joint sliders
- **Robotic Hand** — 5-finger hand with ASL gesture and finger count control

---

## Prerequisites

### 1. Install Docker Desktop

Download and install [Docker Desktop](https://www.docker.com/products/docker-desktop/). Make sure it is running before proceeding.

### 2. Install VNC Viewer
Mac:
```bash
sudo chown -R $(whoami) /opt/homebrew
brew install --cask vnc-viewer
```
Windows:
```wingets
winget install --id RealVNC.VNCViewer -e
```
---

## Step 1 — Build the Docker Image

```bash
cd /path/to/robotics
docker build -t arm-sim .
```

The first build takes several minutes. Subsequent builds use cache.

> You will see a platform warning (`linux/amd64` on `linux/arm64`) — this is harmless. The container runs via Rosetta 2 emulation.

---

## Step 2 — Run the Container
Mac:
```bash
docker run -it \
  --name arm-sim \
  -p 5900:5900 \
  arm-sim
```
Windows:
```
docker run -it --name arm-sim -p 5900:5900 --security-opt seccomp=unconfined arm-sim
```

You will see output like:

```
The VNC desktop is: 0.0.0.0:0
PORT=5900
root@<container-id>:/ros2_ws#
```

You are now **inside the container**.

---

## Step 3 — Connect via VNC

Open a **new Mac terminal tab** and run:
Mac:
```bash
open -a "VNC Viewer"
```

In VNC Viewer, connect to:

```
localhost:5900
```

Leave the password blank if prompted and click **Continue**. A desktop will appear.

Windows:
```
& "C:\Program Files\RealVNC\VNC Viewer\vncviewer.exe" localhost:5900
```
```

> To fit the screen, use the **Scale to fit** button in the VNC Viewer toolbar.

---

## Step 4 — Launch a Simulation

Go back to the **container terminal** and run one of the following:

### Robotic Hand

```bash
ros2 launch arm_simulation robotic_hand.launch.py
```

### UR5 Arm

```bash
ros2 launch arm_simulation view_arm.launch.py
```

The RViz2 window will appear inside the VNC Viewer. You can drag and resize it using the title bar (provided by the Openbox window manager).

---

## Step 5 — Control the Hand

Open a **second terminal into the running container**:

```bash
docker exec -it arm-sim bash
```

### Finger counting (0–5)

```bash
# Fist
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 0}" --once

# 1 finger
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 1}" --once

# 3 fingers
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 3}" --once

# Open hand
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 5}" --once
```

### ASL gestures (single letter)

```bash
# Fist (letter A)
ros2 topic pub /gesture_command std_msgs/msg/String "{data: a}" --once

# L shape
ros2 topic pub /gesture_command std_msgs/msg/String "{data: l}" --once

# Peace sign (letter V)
ros2 topic pub /gesture_command std_msgs/msg/String "{data: v}" --once

# Pinky up (letter I)
ros2 topic pub /gesture_command std_msgs/msg/String "{data: i}" --once

# Neutral / reset
ros2 topic pub /gesture_command std_msgs/msg/String "{data: neutral}" --once
```

All supported gestures: `a` through `z`, `hello`, `neutral`

### ASL letter spelling (via /letter_command)

```bash
ros2 topic pub /letter_command std_msgs/msg/String "{data: r}" --once
```

---

## Dual Hands Simulation

Shows two hands side by side with multiple control modes:

- **Finger counting (0–10)**: Automatically splits across both hands
- **Two-character words**: Right hand shows first character, left hand shows second character
- **Single letters**: Both hands display the same letter
- **Independent control**: Each hand can be controlled separately

### Launch

Rebuild the image first if you haven't since this feature was added:

```bash
docker rm -f arm-sim && docker build -t arm-sim . && docker run -it --name arm-sim -p 5900:5900 arm-sim
```

Inside the container:

```bash
ros2 launch arm_simulation dual_hands.launch.py
```

Connect via VNC Viewer to `localhost:5900` — both hands appear side by side in RViz2.

### Combined finger count (0–10)

Open a second terminal into the container:

```bash
docker exec -it arm-sim bash
```

Then send commands:

```bash
# 8 total → left=5, right=3
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 8}" --once

# 3 total → left=3, right=0
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 3}" --once

# 10 total → left=5, right=5
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 10}" --once
```

### Two-character words (Dual Hand Spelling)

Right hand displays the first character, left hand displays the second character:

```bash
# Word "ab" → right hand: "a", left hand: "b"
ros2 topic pub /letter_command std_msgs/msg/String "{data: ab}" --once

# Word "go" → right hand: "g", left hand: "o"
ros2 topic pub /letter_command std_msgs/msg/String "{data: go}" --once

# Word "it" → right hand: "i", left hand: "t"
ros2 topic pub /letter_command std_msgs/msg/String "{data: it}" --once
```

### Single letters (Both hands display same)

```bash
# Both hands show "a"
ros2 topic pub /letter_command std_msgs/msg/String "{data: a}" --once

# Both hands show "z"
ros2 topic pub /letter_command std_msgs/msg/String "{data: z}" --once
```

### Independent hand control

Each hand can also be controlled separately:

```bash
# Gestures
ros2 topic pub /left/gesture_command std_msgs/msg/String "{data: v}" --once
ros2 topic pub /right/gesture_command std_msgs/msg/String "{data: a}" --once

# ASL letters
ros2 topic pub /left/letter_command std_msgs/msg/String "{data: l}" --once
ros2 topic pub /right/letter_command std_msgs/msg/String "{data: r}" --once

# Finger count per hand
ros2 topic pub /left/finger_count std_msgs/msg/Int32 "{data: 3}" --once
ros2 topic pub /right/finger_count std_msgs/msg/Int32 "{data: 5}" --once
```

### Dual hands topic reference

| Topic                    | Type              | Description                                |
| ------------------------ | ----------------- | ------------------------------------------ |
| `/finger_count`          | `std_msgs/Int32`  | Combined count 0–10, splits automatically  |
| `/left/finger_count`     | `std_msgs/Int32`  | Left hand only (0–5)                       |
| `/right/finger_count`    | `std_msgs/Int32`  | Right hand only (0–5)                      |
| `/left/gesture_command`  | `std_msgs/String` | Gesture on left hand (a–z, hello, neutral) |
| `/right/gesture_command` | `std_msgs/String` | Gesture on right hand                      |
| `/left/letter_command`   | `std_msgs/String` | ASL letter on left hand                    |
| `/right/letter_command`  | `std_msgs/String` | ASL letter on right hand                   |

---

## Humanoid Upper Body Simulation

Shows a realistic human-like upper body with face, head, shoulders, arms, and hands. This is the most realistic representation for demonstrating ASL in real-world contexts.

### Launch

Inside the container:

```bash
ros2 launch arm_simulation humanoid_upper_body.launch.py
```

Connect via VNC Viewer to `localhost:5900` — the humanoid model appears in RViz2 with full body context.

### Model Features

- **Face** with eyes and mouth for natural expression
- **Head** and neck with realistic proportions
- **Shoulders** connecting both arms in human-like positioning
- **Arms** with upper arm, forearm, and wrist segments
- **Hands** with full finger articulation for ASL gestures
- **Torso** in neutral clothing (dark shirt)
- **Skin tones** for realistic appearance

### Control Commands

The humanoid uses the same topics as the dual-hand system:

#### Two-Character Words

Right hand displays the first character, left hand displays the second character:

```bash
# Word "ab" → right hand: "a", left hand: "b"
ros2 topic pub /letter_command std_msgs/msg/String "{data: ab}" --once

# Word "go" → right hand: "g", left hand: "o"
ros2 topic pub /letter_command std_msgs/msg/String "{data: go}" --once

# Word "hi" → right hand: "h", left hand: "i"
ros2 topic pub /letter_command std_msgs/msg/String "{data: hi}" --once

# Word "on" → right hand: "o", left hand: "n"
ros2 topic pub /letter_command std_msgs/msg/String "{data: on}" --once
```

#### Single Letters (Both Hands)

```bash
# Both hands show "a"
ros2 topic pub /letter_command std_msgs/msg/String "{data: a}" --once

# Both hands show "z"
ros2 topic pub /letter_command std_msgs/msg/String "{data: z}" --once
```

#### Combined Finger Count (0–10)

```bash
# 8 total → left=5, right=3
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 8}" --once

# 10 total → left=5, right=5
ros2 topic pub /finger_count std_msgs/msg/Int32 "{data: 10}" --once
```

#### Independent Hand Control

```bash
# Left hand letters
ros2 topic pub /left/letter_command std_msgs/msg/String "{data: l}" --once

# Right hand letters
ros2 topic pub /right/letter_command std_msgs/msg/String "{data: r}" --once

# Gestures on each hand
ros2 topic pub /left/gesture_command std_msgs/msg/String "{data: v}" --once
ros2 topic pub /right/gesture_command std_msgs/msg/String "{data: a}" --once
```

### Use Cases

- **Educational demonstrations** - Show ASL in human context
- **Communication assistance** - Help hearing people understand sign language
- **Real-world presentation** - More realistic than abstract hand models
- **Interactive learning** - Face expressions add communication context
- **Accessibility** - Natural way to represent deaf communication

---

## Container Management

| Task                                    | Command                                                                                                   |
| --------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| Stop the container                      | `docker stop arm-sim`                                                                                     |
| Restart and re-attach                   | `docker start -ai arm-sim`                                                                                |
| Open a second terminal in the container | `docker exec -it arm-sim bash`                                                                            |
| Remove the container (keeps image)      | `docker rm arm-sim`                                                                                       |
| Rebuild after code changes              | `docker rm -f arm-sim && docker build -t arm-sim . && docker run -it --name arm-sim -p 5900:5900 arm-sim` |

---

## ROS Topics Reference

| Topic              | Type                     | Description                         |
| ------------------ | ------------------------ | ----------------------------------- |
| `/finger_count`    | `std_msgs/Int32`         | Set finger count 0–5                |
| `/gesture_command` | `std_msgs/String`        | Named gesture (a–z, hello, neutral) |
| `/letter_command`  | `std_msgs/String`        | Single ASL letter (a–z)             |
| `/joint_states`    | `sensor_msgs/JointState` | Current joint positions (output)    |
| `/current_gesture` | `std_msgs/String`        | Active gesture name (output)        |

```bash
# Useful diagnostics (run inside container)
ros2 topic list          # all active topics
ros2 node list           # all running nodes
ros2 topic echo /current_gesture   # watch active gesture
```

---

## Troubleshooting

**VNC Viewer stuck on connecting**

- Mac's built-in Screen Sharing does not work — use VNC Viewer only

**Blank screen in VNC**

- Nothing has been launched yet — run the launch command in the container terminal

**Can't move RViz window**

- Openbox window manager is included — windows have title bars and can be dragged

**Build fails with `python3-pyaudio` error**

- Already handled — the Dockerfile skips it with `--skip-keys`

**Platform warning during build or run**

- Harmless — the amd64 image runs via Rosetta 2 on Apple Silicon
