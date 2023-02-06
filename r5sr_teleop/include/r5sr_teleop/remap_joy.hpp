#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace r5sr_teleop {

class RemapJoy : public rclcpp::Node {
 public:
  RemapJoy();

 private:
  std::string teleop_mode;
  bool is_emergency_stopped;

  void handle_joy(const sensor_msgs::msg::Joy::SharedPtr joy);
  void handle_is_emergency_stopped(const std_msgs::msg::Bool::SharedPtr joy);
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_emergency_stopped_sub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr teleop_mode_pub;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub;
};
}  // namespace r5sr_teleop