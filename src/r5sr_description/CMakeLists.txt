cmake_minimum_required(VERSION 3.5)
project(r5sr_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz2 REQUIRED)
find_package(realsense2_description REQUIRED)

install(DIRECTORY
  meshes
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()