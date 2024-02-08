cmake_minimum_required(VERSION 3.5)
project(r5sr_bringup)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

find_package(ament_cmake REQUIRED)

ament_package()