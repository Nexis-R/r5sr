cmake_minimum_required(VERSION 3.5)
project(r5sr_teleop)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(r5sr_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()

add_library(joy2command_lib SHARED
  src/joy2command.cpp)
target_include_directories(joy2command_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_nodes(joy2command_lib "r5sr_teleop::Joy2Command")
ament_target_dependencies(joy2command_lib
  rclcpp
  rclcpp_components
  r5sr_interface
  std_srvs
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  control_msgs)
add_executable(joy2command_node src/joy2command_node.cpp)
ament_target_dependencies(joy2command_node rclcpp)
target_link_libraries(joy2command_node joy2command_lib)

ament_auto_add_executable(flir_ax8_rtsp
  include/r5sr_teleop/flir_ax8_rtsp.hpp
  src/flir_ax8_rtsp.cpp)

target_link_libraries(flir_ax8_rtsp ${OpenCV_LIBRARIES})
  
add_executable(alert_battery src/alert_battery.cpp)
ament_target_dependencies(alert_battery rclcpp std_msgs rviz_2d_overlay_msgs)

install(TARGETS
  joy2command_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS
  joy2command_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  alert_battery
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  rqt
  DESTINATION share/${PROJECT_NAME})

ament_auto_package()