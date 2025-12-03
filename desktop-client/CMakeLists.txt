cmake_minimum_required(VERSION 3.16)
project(PrecisionFarmingDesktopClient VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Option to build with ROS2 support
option(USE_ROS2 "Build with ROS2 support" ON)

# Find Qt6
find_package(Qt6 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
    Network
)

# Optional Qt6 components
find_package(Qt6 QUIET COMPONENTS
    Multimedia
    MultimediaWidgets
)

if(Qt6Multimedia_FOUND)
    message(STATUS "Qt6 Multimedia found - Video support enabled")
    set(HAVE_QT_MULTIMEDIA ON)
    add_definitions(-DHAVE_QT_MULTIMEDIA)
else()
    message(STATUS "Qt6 Multimedia not found - Building without video support")
    set(HAVE_QT_MULTIMEDIA OFF)
endif()

# Find ROS2 (optional)
if(USE_ROS2)
    message(STATUS "Building with ROS2 support")
    find_package(ament_cmake QUIET)
    find_package(rclcpp QUIET)
    find_package(std_msgs QUIET)
    find_package(sensor_msgs QUIET)
    find_package(geometry_msgs QUIET)
    
    if(NOT rclcpp_FOUND)
        message(WARNING "ROS2 packages not found. Building without ROS2 support.")
        set(USE_ROS2 OFF)
    else()
        add_definitions(-DUSE_ROS2)
    endif()
else()
    message(STATUS "Building without ROS2 support")
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ros2
    ${CMAKE_CURRENT_SOURCE_DIR}/src/twin
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ui
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ui/widgets
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
)

# Source files
set(SOURCES
    src/main.cpp
    src/core/Application.cpp
    src/core/WidgetManager.cpp
    src/ros2/ROS2Interface.cpp
    src/twin/DigitalTwin.cpp
    src/twin/TwinState.cpp
    src/twin/TwinSimulator.cpp
    src/ui/MainWindow.cpp
    src/ui/widgets/BaseWidget.cpp
    src/ui/widgets/VideoStreamWidget.cpp
    src/ui/widgets/CommandControlWidget.cpp
    src/ui/widgets/MotionControlWidget.cpp
    src/ui/widgets/SensorDataWidget.cpp
    src/ui/widgets/CoordinatesWidget.cpp
    src/ui/widgets/TwinVisualizationWidget.cpp
    src/utils/Logger.cpp
)

set(HEADERS
    src/core/Application.h
    src/core/WidgetManager.h
    src/ros2/ROS2Interface.h
    src/twin/DigitalTwin.h
    src/twin/TwinState.h
    src/twin/TwinSimulator.h
    src/ui/MainWindow.h
    src/ui/widgets/BaseWidget.h
    src/ui/widgets/VideoStreamWidget.h
    src/ui/widgets/CommandControlWidget.h
    src/ui/widgets/MotionControlWidget.h
    src/ui/widgets/SensorDataWidget.h
    src/ui/widgets/CoordinatesWidget.h
    src/ui/widgets/TwinVisualizationWidget.h
    src/utils/Logger.h
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link Qt libraries
target_link_libraries(${PROJECT_NAME}
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Network
)

# Link optional Qt Multimedia
if(HAVE_QT_MULTIMEDIA)
    target_link_libraries(${PROJECT_NAME}
        Qt6::Multimedia
        Qt6::MultimediaWidgets
    )
endif()

# Link ROS2 libraries (if available)
if(USE_ROS2)
    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
    )
endif()

# Installation
install(TARGETS ${PROJECT_NAME}
    DESTINATION bin
)

# Only use ament_package if ROS2 is available
if(USE_ROS2 AND ament_cmake_FOUND)
    ament_package()
endif()
