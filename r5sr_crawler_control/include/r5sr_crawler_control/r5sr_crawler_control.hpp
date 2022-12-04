#pragma once

#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace crawler_control{

class Crawler_Control : public rclcpp::Node {
 public:
  Crawler_Control();

 private:
  double wheel_diameter;
  double wheel_base;
  void handle_joy(const sensor_msgs::msg::Joy::SharedPtr joy);
  void handle_twist(const geometry_msgs::msg::Twist::SharedPtr twist);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr crawler_left_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr crawler_right_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr flipper_left_front_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr flipper_right_front_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr flipper_left_back_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr flipper_right_back_pub;
};

}  // namespace crawler_control
