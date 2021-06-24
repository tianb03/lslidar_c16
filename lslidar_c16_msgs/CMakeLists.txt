cmake_minimum_required(VERSION 3.5)

project(lslidar_c16_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LslidarC16Layer.msg"
  "msg/LslidarC16Packet.msg"
  "msg/LslidarC16Point.msg"
  "msg/LslidarC16Scan.msg"
  "msg/LslidarC16Sweep.msg"
  "msg/LslidarC16ScanUnified.msg"
  DEPENDENCIES std_msgs sensor_msgs
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

