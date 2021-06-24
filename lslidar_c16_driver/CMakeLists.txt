cmake_minimum_required(VERSION 3.5)
project(lslidar_c16_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lslidar_c16_msgs REQUIRED)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(include)

add_library(lslidar_input src/input.cc)
target_link_libraries(lslidar_input
    ${libpcap_LIBRARIES})
ament_target_dependencies(lslidar_input
  rclcpp
  lslidar_c16_msgs
)

add_library(lslidar_c16_driver src/lslidar_c16_driver.cpp)
ament_target_dependencies(lslidar_c16_driver
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_components
  tf2_ros
  lslidar_c16_msgs
)
target_link_libraries(lslidar_c16_driver
  lslidar_input)

add_executable(lslidar_c16_driver_node src/lslidar_c16_driver_node.cpp)
ament_target_dependencies(lslidar_c16_driver_node
  rclcpp
)
target_link_libraries(lslidar_c16_driver_node
    lslidar_c16_driver
    lslidar_input
    ${libpcap_LIBRARIES}
    )

# install runtime and library files
install(TARGETS lslidar_c16_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS lslidar_c16_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS lslidar_input
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_nodes(lslidar_c16_driver
  "lslidar_c16_driver::lslidarDriver")

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
# install(PROGRAMS src/vdump
#   DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(lslidar_c16_driver)

if(BUILD_TESTING)
endif()

ament_package()
