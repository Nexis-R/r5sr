cmake_minimum_required(VERSION 3.8)

project(r5sr_cloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cpprestsdk REQUIRED)

add_library(rms_plant SHARED src/rms_plant.cpp)
target_link_libraries(rms_plant rclcpp::rclcpp rclcpp_components::component
                      cpprestsdk::cpprest)
target_include_directories(
  rms_plant
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")
rclcpp_components_register_node(rms_plant PLUGIN "r5sr_cloud::RmsPlant"
                                EXECUTABLE rms_plant_node)

add_library(rms_robot SHARED src/rms_robot.cpp)
target_link_libraries(rms_robot rclcpp::rclcpp rclcpp_components::component
                      cpprestsdk::cpprest)
target_include_directories(
  rms_robot
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")
rclcpp_components_register_node(rms_robot PLUGIN "r5sr_cloud::RmsRobot"
                                EXECUTABLE rms_robot_node)

install(
  TARGETS rms_plant rms_robot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
