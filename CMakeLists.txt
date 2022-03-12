cmake_minimum_required(VERSION 3.8)
project(spatz_manual_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

set(SOURCES
        src/main.cpp
        src/ManualControlNode.cpp)

add_executable(ManualControlNode ${SOURCES})
ament_target_dependencies(ManualControlNode rclcpp spatz_interfaces sensor_msgs)

install(TARGETS ManualControlNode
        DESTINATION lib/${PROJECT_NAME})

ament_package()
