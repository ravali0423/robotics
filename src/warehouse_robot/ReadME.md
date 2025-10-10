source install/setup.bash

# Generate new waypoints
source install/setup.bash && python3 src/warehouse_robot/scripts/generate_new_waypoints.py && colcon build --packages-select warehouse_robot && ros2 launch warehouse_robot custom_warehouse.launch.py

# Launch the project simulation
ros2 launch warehouse_robot custom_warehouse.launch.py

# In a new terminal:
source install/setup.bash

# Move forward
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" --once

# Turn left  
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}" --once

# Turn right
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{angular: {z: -1.0}}" --once

# Stop
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{}" --once

# The Python script now works correctly:
source install/setup.bash
ros2 run warehouse_robot car_controller.py forward
ros2 run warehouse_robot car_controller.py demo

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/warehouse_car/cmd_vel