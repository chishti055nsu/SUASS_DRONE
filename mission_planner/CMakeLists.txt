cmake_minimum_required(VERSION 3.8)
project(mission_planner)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  mission_planner/mission_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_node
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

