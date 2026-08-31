cmake_minimum_required(VERSION 3.8)
project(drone_vision)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  drone_vision/vision_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vision_node
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

