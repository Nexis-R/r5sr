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
find_package(std_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(remap_joy_lib SHARED
  include/r5sr_teleop/remap_joy.hpp
  src/remap_joy.cpp)

ament_auto_add_executable(remap_joy
  src/remap_joy_node.cpp)

ament_auto_add_executable(flir_ax8_rtsp
  include/r5sr_teleop/flir_ax8_rtsp.hpp
  src/flir_ax8_rtsp.cpp)

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