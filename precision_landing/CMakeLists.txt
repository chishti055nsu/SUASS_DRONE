cmake_minimum_required(VERSION 3.8)
project(precision_landing)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  precision_landing/precision_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME precision_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

