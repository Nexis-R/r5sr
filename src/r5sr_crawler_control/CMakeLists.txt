cmake_minimum_required(VERSION 3.5)
project(r5sr_crawler_control)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(r5sr_crawler_control SHARED
  src/r5sr_crawler_control.cpp
)
target_include_directories(r5sr_crawler_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_nodes(r5sr_crawler_control "crawler_control::Crawler_Control")

ament_target_dependencies(r5sr_crawler_control
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  rclcpp_components
)

target_link_libraries(r5sr_crawler_control
)

add_executable(r5sr_crawler_control_node
  src/r5sr_crawler_control_node.cpp
)
ament_target_dependencies(r5sr_crawler_control_node
  rclcpp
)
target_link_libraries(r5sr_crawler_control_node
  r5sr_crawler_control
)

install(TARGETS
  r5sr_crawler_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  r5sr_crawler_control_node
  DESTINATION lib/${PROJECT_NAME}
) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
