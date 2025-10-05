cmake_minimum_required(VERSION 3.20)
project(arm_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install Python scripts
install(PROGRAMS
  scripts/hand_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
