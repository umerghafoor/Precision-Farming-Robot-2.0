cmake_minimum_required(VERSION 3.10)
project(mqtt_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(mqtt_bridge)

install(PROGRAMS
  mqtt_bridge/mqtt_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mqtt_bridge_node
)

ament_package()
