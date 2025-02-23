cmake_minimum_required(VERSION 3.8)
project(six_axis_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)

# Install directories: URDF files, launch files, and config files (if any)
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the scripts as executables.
install(PROGRAMS
  scripts/robot_arm_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()



