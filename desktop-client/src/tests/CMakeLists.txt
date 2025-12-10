cmake_minimum_required(VERSION 3.10)
project(DesktopClientTests)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(GTest REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Network)
find_package(Qt6 QUIET COMPONENTS Multimedia MultimediaWidgets)

option(USE_ROS2 "Build with ROS2 support" ON)
if(USE_ROS2)
    find_package(ament_cmake QUIET)
    find_package(rclcpp QUIET)
    find_package(std_msgs QUIET)
    find_package(sensor_msgs QUIET)
    find_package(geometry_msgs QUIET)
endif()

# Get the absolute path to the src directory
get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../" ABSOLUTE)

include_directories(
    ${GTEST_INCLUDE_DIRS}
    ${SRC_DIR}
    ../ui
    ../ui/widgets
    ../ros2
    ../twin
    ../utils
    ../core
)


# Include all main source files for linkage
set(MAIN_SOURCES
    ../core/Application.cpp
    ../core/WidgetManager.cpp
    ../ros2/ROS2Interface.cpp
    ../twin/DigitalTwin.cpp
    ../twin/TwinState.cpp
    ../twin/TwinSimulator.cpp
    ../ui/MainWindow.cpp
    ../ui/widgets/BaseWidget.cpp
    ../ui/widgets/VideoStreamWidget.cpp
    ../ui/widgets/CommandControlWidget.cpp
    ../ui/widgets/MotionControlWidget.cpp
    ../ui/widgets/SensorDataWidget.cpp
    ../ui/widgets/CoordinatesWidget.cpp
    ../ui/widgets/TwinVisualizationWidget.cpp
    ../utils/Logger.cpp
)

set(MAIN_HEADERS
    ../core/Application.h
    ../core/WidgetManager.h
    ../ros2/ROS2Interface.h
    ../twin/DigitalTwin.h
    ../twin/TwinState.h
    ../twin/TwinSimulator.h
    ../ui/MainWindow.h
    ../ui/widgets/BaseWidget.h
    ../ui/widgets/VideoStreamWidget.h
    ../ui/widgets/CommandControlWidget.h
    ../ui/widgets/MotionControlWidget.h
    ../ui/widgets/SensorDataWidget.h
    ../ui/widgets/CoordinatesWidget.h
    ../ui/widgets/TwinVisualizationWidget.h
    ../utils/Logger.h
)

add_executable(runDesktopClientTests
    test_main.cpp
    test_widget_manager.cpp
    test_logger.cpp
    test_ros2_interface.cpp
    ${MAIN_SOURCES}
    ${MAIN_HEADERS}
)

target_link_libraries(runDesktopClientTests
    ${GTEST_LIBRARIES}
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Network
    pthread
)

if(Qt6Multimedia_FOUND)
    target_link_libraries(runDesktopClientTests Qt6::Multimedia Qt6::MultimediaWidgets)
endif()

if(USE_ROS2 AND rclcpp_FOUND)
    ament_target_dependencies(runDesktopClientTests rclcpp std_msgs sensor_msgs geometry_msgs)
endif()
