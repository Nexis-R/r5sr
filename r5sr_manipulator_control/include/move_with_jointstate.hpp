#pragma once

#include <memory>
#include <string_view>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace r5sr_manipulator_control {
enum Model {
  H54,
  H42,
  XH430,

};
//{model, {torque enable, goal position, pulse per rev}}
const static std::map<Model, std::tuple<uint16_t, uint16_t, uint16_t>>
    model_addr_map{{H54, {562, 596, 501'923}},
                   {H42, {562, 596, 303'750}},
                   {XH430, {64, 116, 4'096}}};

//{jointname, {model, id, offset, coef}}
const static std::map<std::string, std::tuple<Model, uint8_t, float, float>>
    jointname_id_model_map{{"body1_joint", {H54, 2, 0.0, -1.0}},
                           {"body2_joint", {H54, 3, 163.0, -1.0}},
                           {"body3_joint", {H54, 4, -150.0, -1.0}},
                           {"body4_joint", {H42, 5, -45.0, -1.0}},
                           {"body5_joint", {H42, 6, 0.0, -1.0}},
                           {"body6_joint", {H42, 7, 0.0, 1.0}},
                           {"vision_arm_body1_joint", {H42, 9, -50.0, -1.0}},
                           {"vision_arm_body2_joint", {XH430, 10, 175.0, -1.0}},
                           {"vision_arm_body3_joint", {XH430, 11, 180.0, -1.0}}};

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