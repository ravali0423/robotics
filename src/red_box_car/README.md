# Red Box Car - Autonomous Object Retrieval Robot

This ROS2 package implements an autonomous car with a camera and gripper that can:
1. Search for red boxes using computer vision
2. Navigate towards detected red boxes
3. Pick up the red box using a gripper mechanism
4. Return to the starting position
5. Drop the red box at the home location

## Package Structure

```
red_box_car/
├── red_box_car/           # Python source code
│   ├── vision_node.py         # Computer vision for red box detection
│   ├── navigation_controller.py # Movement control towards target
│   ├── pickup_controller.py   # Gripper control for object manipulation
│   └── state_machine.py       # Main mission orchestrator
├── launch/                # Launch files
│   ├── red_box_car_simulation.launch.py  # Full simulation launch
│   └── red_box_car_control.launch.py     # Control nodes only
├── config/                # Configuration files
│   └── red_box_car_config.yaml
├── urdf/                  # Robot description
│   └── red_box_car.urdf
├── worlds/                # Gazebo world files
│   └── red_box_world.world
└── package.xml           # Package configuration
```

## Nodes

### vision_node
- **Purpose**: Detects red boxes in camera feed using OpenCV
- **Subscriptions**: `/camera/image_raw`
- **Publications**: 
  - `/red_box_detected` (Bool)
  - `/red_box_position` (Point) 
  - `/red_box_distance` (Float32)
  - `/debug_image` (Image)

### navigation_controller
- **Purpose**: Controls robot movement to approach detected targets
- **Subscriptions**: 
  - `/red_box_detected`
  - `/red_box_position`
  - `/red_box_distance`
  - `/navigation_command`
- **Publications**: 
  - `/cmd_vel` (Twist)
  - `/navigation_status` (String)

### pickup_controller
- **Purpose**: Controls gripper for object manipulation
- **Subscriptions**: 
  - `/pickup_command`
  - `/gripper_controller/state`
- **Publications**:
  - `/gripper_controller/command` (Float64)
  - `/pickup_status` (String)
  - `/object_grasped` (Bool)

### state_machine
- **Purpose**: Orchestrates the complete mission sequence
- **Subscriptions**: 
  - `/red_box_detected`
  - `/navigation_status`
  - `/pickup_status`
  - `/object_grasped`
  - `/mission_command`
- **Publications**:
  - `/navigation_command` (String)
  - `/pickup_command` (String)
  - `/mission_status` (String)

## Usage

### Simulation
```bash
# Build the package
colcon build --packages-select red_box_car

# Source the workspace
source install/setup.bash

# Launch full simulation
ros2 launch red_box_car red_box_car_simulation.launch.py

# Start the mission (in another terminal)
ros2 topic pub /mission_command std_msgs/String "data: start" --once
```

### Real Robot
```bash
# Launch control nodes only
ros2 launch red_box_car red_box_car_control.launch.py

# Start the mission
ros2 topic pub /mission_command std_msgs/String "data: start" --once
```

## Mission Commands

- `start` - Begin the red box retrieval mission
- `stop` - Stop the current mission
- `reset` - Reset to idle state

## Mission States

1. **IDLE** - Waiting for start command
2. **SEARCHING** - Rotating to find red boxes
3. **APPROACHING** - Moving towards detected red box
4. **PICKING_UP** - Grabbing the red box with gripper
5. **RETURNING** - Navigating back to home position
6. **DROPPING** - Releasing the object at home
7. **COMPLETED** - Mission successful
8. **FAILED** - Mission failed (timeout or error)

## Configuration

Edit `config/red_box_car_config.yaml` to adjust:
- HSV color ranges for red detection
- Navigation speeds and tolerances
- Gripper positions and timing
- Mission timeouts

## Dependencies

- ROS2 (tested on Humble)
- OpenCV for Python
- Gazebo (for simulation)
- cv_bridge
- tf2_ros

## Robot Hardware Requirements

For real robot deployment:
- Differential drive mobile base with `/cmd_vel` interface
- Camera publishing to `/camera/image_raw`
- Gripper with position control interface
- Odometry for navigation (recommended)

## Troubleshooting

1. **No red box detected**: Adjust HSV color ranges in config file
2. **Robot not moving**: Check `/cmd_vel` topic connection
3. **Gripper not working**: Verify gripper controller configuration
4. **Vision processing slow**: Reduce image resolution or processing rate

## Future Improvements

- Add obstacle avoidance
- Implement SLAM for better navigation
- Add multiple object detection
- Improve gripper force feedback
- Add voice commands or GUI interface