cmake_minimum_required(VERSION 3.5)
project(r5sr_moveit_teleop)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_components
    std_msgs
    std_srvs
    sensor_msgs
    control_msgs
    control_toolbox
    trajectory_msgs
    moveit_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen)

add_library(sub_manipulator_control SHARED src/sub_manipulator_control.cpp)
ament_target_dependencies(sub_manipulator_control
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(sub_manipulator_control
                                 "r5sr_moveit_teleop::SubManipulatorControl")

add_library(sub_manipulator_simple_control SHARED
            src/sub_manipulator_control_simple.cpp)
ament_target_dependencies(sub_manipulator_simple_control
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(
  sub_manipulator_simple_control
  "r5sr_moveit_teleop::SubManipulatorSimpleControl")

add_library(preset_pose SHARED src/preset_pose.cpp)
ament_target_dependencies(preset_pose ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(preset_pose "r5sr_moveit_teleop::PresetPose")

install(
  TARGETS sub_manipulator_control
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(
  TARGETS sub_manipulator_simple_control
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(
  TARGETS preset_pose
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
