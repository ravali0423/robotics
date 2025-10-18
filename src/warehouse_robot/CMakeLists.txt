cmake_minimum_required(VERSION 3.8)
project(warehouse_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  launch
  worlds
  rviz
  config
  models
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/car_controller.py
  scripts/robot_controller.py
  scripts/manual_robot_controller.py
  scripts/web_joystick_controller.py
  scripts/generate_waypoints.py
  scripts/mission_log_manager.py
  scripts/show_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional files
install(FILES
  scripts/generate_new_waypoints.py
  DESTINATION share/${PROJECT_NAME}/scripts/
)

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

ament_package()
