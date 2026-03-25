#!/bin/bash
# Start virtual display
Xvfb :1 -screen 0 1280x800x24 &
sleep 1

# Start VNC server (no password, accessible on port 5900)
x11vnc -display :1 -nopw -listen 0.0.0.0 -forever -quiet &

export DISPLAY=:1

# Start window manager so windows have title bars and can be moved
openbox &
sleep 1

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

exec /bin/bash
