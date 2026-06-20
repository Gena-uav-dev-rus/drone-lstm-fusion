cmake_minimum_required(VERSION 3.8)
project(lstm_noise_estimator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  lstm_noise_estimator/gps_noise_node.py
  lstm_noise_estimator/vio_noise_node.py
  lstm_noise_estimator/depth_noise_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
