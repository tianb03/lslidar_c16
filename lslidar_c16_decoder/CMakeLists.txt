cmake_minimum_required(VERSION 3.5)
project(lslidar_c16_decoder)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lslidar_c16_driver REQUIRED)
find_package(lslidar_c16_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

include_directories(include ${PCL_COMMON_INCLUDE_DIRS})
include_directories(
        include
        ${YAML_CPP_INCLUDE_DIRS}
)

add_library(lslidar_data src/rawdata.cc)
ament_target_dependencies(lslidar_data
  angles
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  lslidar_c16_msgs
)
target_link_libraries(lslidar_data
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES}
        yaml-cpp)

add_library(lslidar_point src/convert.cc)
ament_target_dependencies(lslidar_point
  angles
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  lslidar_c16_msgs
)
target_link_libraries(lslidar_point
  lslidar_data
)
install(TARGETS lslidar_point
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# rclcpp_components_register_nodes(lslidar_point
#   "lslidar_c16_decoder::Convert")

add_executable(lslidar_c16_decoder_node src/cloud_node.cc)
ament_target_dependencies(lslidar_c16_decoder_node
  rclcpp
  lslidar_c16_msgs
)
target_link_libraries(lslidar_c16_decoder_node
  lslidar_point
)
install(TARGETS lslidar_c16_decoder_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(lslidar_c16_driver_node_composed
  src/lslidar_c16_driver_node_composed.cpp)
target_link_libraries(lslidar_c16_driver_node_composed
  lslidar_point
)
ament_target_dependencies(lslidar_c16_driver_node_composed
  rclcpp
  lslidar_c16_driver
  lslidar_c16_msgs
)
install(TARGETS lslidar_c16_driver_node_composed
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

ament_package()
