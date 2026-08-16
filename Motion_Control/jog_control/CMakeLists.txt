cmake_minimum_required(VERSION 3.8)
project(jog_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dss_ros2_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(JogControlNode src/JogControlNode.cpp)
ament_target_dependencies(JogControlNode rclcpp dss_ros2_bridge ament_index_cpp)
target_link_libraries(JogControlNode Qt5::Widgets)

install(TARGETS JogControlNode DESTINATION lib/${PROJECT_NAME})

ament_package()
