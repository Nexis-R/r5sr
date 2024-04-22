#pragma once

#include <memory>
#include <rclcpp/timer.hpp>
#include <string_view>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

namespace r5sr_manipulator_control
{
enum Model {
  H54,
  H42,
  XH430,

};
//{model, {torque enable, goal position, pulse per rev}}
const static std::map<Model, std::tuple<uint16_t, uint16_t, int32_t>> model_addr_map{
  {H54, {562, 596, 501'923}}, {H42, {562, 596, 303'750}}, {XH430, {64, 116, 4'096}}};

//{jointname, {model, id, offset, coef}}
const static std::map<std::string, std::tuple<Model, uint8_t, float, float>> jointname_id_model_map{
  {"body_joint", {H54, 2, -15.0, -(72.0 / 36.0)}},
  {"shoulder_joint", {H54, 3, 167.0, -(48.0 / 24.0)}},
  {"elbow_joint", {H54, 4, -152.0, -(36.0 / 24.0)}},
  {"forearm_joint", {H42, 5, -45.0, (24.0 / 20.0)}},
  {"wrist_yaw_joint", {H42, 6, 0.0, 1.0}},
  {"wrist_roll_joint", {H42, 7, 0.0, 1.0}},
  {"overhead_elbow_joint", {H42, 9, 35.0, 1.0}},
  {"overhead_wrist_yaw_joint", {XH430, 10, 130.0, -1.0}},
  {"overhead_wrist_pitch_joint", {XH430, 11, 180.0, 1.0}},
  {"hand", {XH430, 8, 0.0, 0.0}}};

class MoveWithJointState : public rclcpp::Node
{
public:
  MoveWithJointState();

private:
  int32_t hand_pulse;
  std::tuple<float, float, float> vision_angle;

  std::unique_ptr<dynamixel::PortHandler> portHandler;
  std::unique_ptr<dynamixel::PacketHandler> packetHandler;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_command_sub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hand_current_pub;
  rclcpp::TimerBase::SharedPtr status_pub_timer;

  void handle_joint_state(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  void handle_hand_command(const std_msgs::msg::Float32::SharedPtr command);
  void status_pub_callback();
};
}  // namespace r5sr_manipulator_control