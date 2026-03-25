# Arm Simulation — Docker Setup on Mac (Apple Silicon)

ROS 2 Jazzy robotic arm simulation running in Docker, visualized via VNC on Mac.

Two simulations included:
- **UR5 Industrial Arm** — 6-DOF arm with interactive joint sliders
- **Robotic Hand** — 5-finger hand with ASL gesture and finger count control

---

## Prerequisites

### 1. Install Docker Desktop
Download and install [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop/). Make sure it is running before proceeding.

### 2. Install VNC Viewer
```bash
sudo chown -R $(whoami) /opt/homebrew
brew install --cask vnc-viewer
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

```bash
docker run -it \
  --name arm-sim \
  -p 5900:5900 \
  arm-sim
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
```bash
open -a "VNC Viewer"
```

In VNC Viewer, connect to:
```
localhost:5900
```

Leave the password blank if prompted and click **Continue**. A desktop will appear.

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

## Container Management

| Task | Command |
|------|---------|
| Stop the container | `docker stop arm-sim` |
| Restart and re-attach | `docker start -ai arm-sim` |
| Open a second terminal in the container | `docker exec -it arm-sim bash` |
| Remove the container (keeps image) | `docker rm arm-sim` |
| Rebuild after code changes | `docker rm -f arm-sim && docker build -t arm-sim . && docker run -it --name arm-sim -p 5900:5900 arm-sim` |

---

## ROS Topics Reference

| Topic | Type | Description |
|-------|------|-------------|
| `/finger_count` | `std_msgs/Int32` | Set finger count 0–5 |
| `/gesture_command` | `std_msgs/String` | Named gesture (a–z, hello, neutral) |
| `/letter_command` | `std_msgs/String` | Single ASL letter (a–z) |
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions (output) |
| `/current_gesture` | `std_msgs/String` | Active gesture name (output) |

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
