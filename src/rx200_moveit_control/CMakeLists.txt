cmake_minimum_required(VERSION 3.5)
project(rx200_moveit_control)

# # Default to C++14 (needed even if using Python nodes)
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts (executables)
install(
  PROGRAMS
    scripts/rx200_moveit_client
    scripts/keyboard_gui
    scripts/vision
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()