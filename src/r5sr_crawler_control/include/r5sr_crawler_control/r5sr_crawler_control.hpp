#pragma once

#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace crawler_control
{

class Crawler_Control : public rclcpp::Node
{
public:
  explicit Crawler_Control(const rclcpp::NodeOptions& options);

private:
  double wheel_diameter;
  double wheel_base;
  double flipper_gear_ratio;
  double crawler_gear_ratio;

  void handle_twist(const geometry_msgs::msg::Twist::SharedPtr twist);
  void handle_jointstate(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate_sub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr crawler_left_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr crawler_right_pub;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr flipper_left_front_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr flipper_right_front_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr flipper_left_back_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr flipper_right_back_pub;
};

}  // namespace crawler_control
