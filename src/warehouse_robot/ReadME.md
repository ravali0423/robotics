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
ros2 run warehouse_robot robot_controller.py stop      # Stop the robot immediately

# 🎯 Move to specific XY coordinates:
ros2 run warehouse_robot robot_controller.py goto 2.0 3.0    # Move to X=2.0, Y=3.0
ros2 run warehouse_robot robot_controller.py goto -1.5 4.2   # Move to X=-1.5, Y=4.2

# 🎮 Manual Control Options:
# Move forward
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" --once

# Turn left  
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}" --once

# Turn right
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{angular: {z: -1.0}}" --once

# Stop
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{}" --once

# Move to specific XY coordinates (autonomous navigation):
# This uses the robot controller to navigate to any XY location
python3 -c "
import rclpy
from warehouse_robot.scripts.robot_controller import WarehouseRobotController
rclpy.init()
controller = WarehouseRobotController()
controller.move_to_target(2.0, 3.0)  # Move to X=2.0, Y=3.0
controller.destroy_node()
rclpy.shutdown()
"

# Alternative: Direct cmd_vel for manual XY movement (requires manual calculation)
# For X=2.0, Y=3.0 from origin, calculate angle and distance first:
# Distance = sqrt(2²+3²) = 3.61, Angle = atan2(3,2) = 0.98 rad
# First rotate to face target, then move forward
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.98}}" --once
# Wait, then move forward
ros2 topic pub /model/warehouse_car/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" --once

# 🕹️ Basic Car Controller (Original):
source install/setup.bash
ros2 run warehouse_robot car_controller.py forward
ros2 run warehouse_robot car_controller.py demo

# 🎮 Keyboard Teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/warehouse_car/cmd_vel

# 📍 View Current Waypoints:
python3 src/warehouse_robot/scripts/show_waypoints.py