#pragma once

#include <memory>
#include <string_view>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace r5sr_manipulator_control {
enum Model {
  MX64,
};
//{model, {torque enable, goal position, pulse per rev}}
const static std::map<Model, std::tuple<uint16_t, uint16_t, uint16_t>>
    model_addr_map{{MX64, {64, 116, 4096}}};

//{model, {torque enable, goal position, pulse per rev}}
const static std::map<std::string, std::tuple<Model, uint8_t>>
    jointname_id_model_map{
        {"body0_joint_body", {MX64, 2}}, {"body1_joint", {MX64, 3}},
        {"body2_joint", {MX64, 4}},      {"body3_joint", {MX64, 5}},
        {"body4_joint", {MX64, 6}},      {"body5_joint", {MX64, 7}},
        {"body6_joint", {MX64, 8}},      {"vision_arm_body1_joint", {MX64, 9}},
    };

class MoveWithJointState : public rclcpp::Node {
 public:
  MoveWithJointState();

 private:
  std::unique_ptr<dynamixel::PortHandler> portHandler;
  std::unique_ptr<dynamixel::PacketHandler> packetHandler;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;

  void handle_joint_state(
      const sensor_msgs::msg::JointState::SharedPtr joint_state);
};
}  // namespace r5sr_manipulator_control