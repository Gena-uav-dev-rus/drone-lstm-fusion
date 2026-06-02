cmake_minimum_required(VERSION 3.8)
project(drone_bringup)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  launch/gazebo_mono_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gazebo_mono_driver
)
ament_package()
