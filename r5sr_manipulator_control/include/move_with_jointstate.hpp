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

//{model, {torque enable, goal position, pulse per rev}}
const static std::map<std::string, std::tuple<Model, uint8_t>>
    jointname_id_model_map{{"body0_joint_yaw", {H54, 2}},
                           {"body1_joint", {H54, 3}},
                           {"body2_joint", {H54, 4}},
                           {"body3_joint", {H42, 5}},
                           {"body4_joint", {H42, 9}},
                           {"body5_joint", {H42, 6}},
                           {"body6_joint", {H42, 7}},
                           {"vision_arm_body1_joint", {H42, 9}},
                           {"vision_arm_body2_joint", {XH430, 10}},
                           {"vision_arm_body3_joint", {XH430, 11}}};

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