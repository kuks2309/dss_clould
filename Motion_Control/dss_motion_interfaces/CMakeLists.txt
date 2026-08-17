cmake_minimum_required(VERSION 3.8)
project(dss_motion_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MotionStanley.action"
  "action/MotionPurePursuit.action"
  DEPENDENCIES nav_msgs
)

ament_package()
