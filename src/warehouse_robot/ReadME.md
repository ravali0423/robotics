source install/setup.bash

# Generate new waypoints (includes start, package pickup, and destination)
source install/setup.bash && python3 src/warehouse_robot/scripts/generate_new_waypoints.py && colcon build --packages-select warehouse_robot && ros2 launch warehouse_robot custom_warehouse.launch.py

# Launch the project simulation (uses existing waypoints)
ros2 launch warehouse_robot custom_warehouse.launch.py

# Warehouse Robot Package Delivery Commands:
# In a new terminal:
source install/setup.bash

# 🤖 Enhanced Robot Controller (Package Delivery System):
ros2 run warehouse_robot robot_controller.py pickup    # Go to package and pick it up
ros2 run warehouse_robot robot_controller.py deliver   # Deliver package to destination  
ros2 run warehouse_robot robot_controller.py return    # Return to start position
ros2 run warehouse_robot robot_controller.py mission   # Execute full pickup → deliver → return sequence
ros2 run warehouse_robot robot_controller.py status    # Show current robot status

# 🎮 Manual Control Options:
# Move forward
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" --once

# Turn left  
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}" --once

# Turn right
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{angular: {z: -1.0}}" --once

# Stop
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{}" --once

# 🕹️ Basic Car Controller (Original):
source install/setup.bash
ros2 run warehouse_robot car_controller.py forward
ros2 run warehouse_robot car_controller.py demo

# 🎮 Keyboard Teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/warehouse_car/cmd_vel

# 📍 View Current Waypoints:
python3 src/warehouse_robot/scripts/show_waypoints.py