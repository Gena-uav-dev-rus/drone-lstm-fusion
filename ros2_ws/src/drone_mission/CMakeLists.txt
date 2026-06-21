cmake_minimum_required(VERSION 3.8)
project(drone_mission)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  drone_mission/mission.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
