cmake_minimum_required(VERSION 3.10)
project(camera_sensor)

find_package(ament_cmake REQUIRED)

# Install Python source files
install(DIRECTORY camera_sensor/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install executable script
install(PROGRAMS
  camera_sensor/camera_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_node
)

ament_package()