cmake_minimum_required(VERSION 3.10)
project(ground_truth_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(${PROJECT_NAME} SHARED src/GroundTruthPlugin.cc)

target_link_libraries(${PROJECT_NAME}
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

ament_package()
