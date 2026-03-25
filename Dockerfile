FROM --platform=linux/amd64 osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-ur-description \
    python3-colcon-common-extensions \
    xvfb \
    x11vnc \
    openbox \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY src/arm_simulation src/arm_simulation/

RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys 'python3-pyaudio python3-speechrecognition' && \
    colcon build --symlink-install"

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

COPY start.sh /start.sh
RUN chmod +x /start.sh

EXPOSE 5900

CMD ["/start.sh"]
