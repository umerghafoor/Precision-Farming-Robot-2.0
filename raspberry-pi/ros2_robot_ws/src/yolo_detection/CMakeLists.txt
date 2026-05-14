cmake_minimum_required(VERSION 3.10)
project(yolo_detection)

find_package(ament_cmake REQUIRED)

# Install Python modules
ament_python_install_package(yolo_detection)

# Install Python executables
install(PROGRAMS
  yolo_detection/yolo_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_detection_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
