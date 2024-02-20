cmake_minimum_required(VERSION 3.8)
project(r5sr_interface)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME} "msg/ManipulatorSpeed.msg" "msg/CrawlerSpeed.msg"
  "msg/CrawlerMode.msg" "msg/TeleopMode.msg" "msg/OperationMode.msg")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
