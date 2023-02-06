#include "r5sr_crawler_control/r5sr_crawler_control.hpp"

using namespace crawler_control;

using std::placeholders::_1;

Crawler_Control::Crawler_Control() : Node("r5sr_crawler_control") {
  this->declare_parameter("wheel_diameter", 1.0);
  this->declare_parameter("wheel_base", 1.0);

  wheel_diameter = this->get_parameter("wheel_diameter").as_double();
  wheel_base = this->get_parameter("wheel_base").as_double();

  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1, std::bind(&Crawler_Control::handle_joy, this, _1));
  twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1, std::bind(&Crawler_Control::handle_twist, this, _1));
  crawler_left_pub =
      this->create_publisher<std_msgs::msg::Float32>("command_crawler_left", 1);
  crawler_right_pub = this->create_publisher<std_msgs::msg::Float32>(
      "command_crawler_right", 1);

  flipper_left_front_pub = this->create_publisher<std_msgs::msg::Float32>(
      "command_flipper_left_front", 1);
  flipper_right_front_pub = this->create_publisher<std_msgs::msg::Float32>(
      "command_flipper_right_front", 1);
  flipper_left_back_pub = this->create_publisher<std_msgs::msg::Float32>(
      "command_flipper_left_back", 1);
  flipper_right_back_pub = this->create_publisher<std_msgs::msg::Float32>(
      "command_flipper_right_back", 1);
}

void Crawler_Control::handle_joy(const sensor_msgs::msg::Joy::SharedPtr joy) {
  std_msgs::msg::Float32 flipper_speed;
  std_msgs::msg::Float32 speed_zero;

  speed_zero.set__data(0.0);
  const float gear_ratio = 4500.0;

  flipper_speed.set__data(joy->axes.at(13) * gear_ratio);
  flipper_left_front_pub->publish(flipper_speed);
  
  flipper_speed.set__data(joy->axes.at(14) * gear_ratio);
  flipper_right_front_pub->publish(flipper_speed);

  flipper_speed.set__data(joy->axes.at(15) * gear_ratio);
  flipper_left_back_pub->publish(flipper_speed);

  flipper_speed.set__data(joy->axes.at(16) * gear_ratio);
  flipper_right_back_pub->publish(flipper_speed);
}

void Crawler_Control::handle_twist(
    const geometry_msgs::msg::Twist::SharedPtr twist) {
  const float vx = twist->linear.x;
  const float va = twist->angular.z;

  const float gear_ratio = 60.0;

  std_msgs::msg::Float32 left_speed;
  std_msgs::msg::Float32 right_speed;
  left_speed.set__data((((vx * 60) / (wheel_diameter * M_PI)) +
                        (((60 * va) / (2 * M_PI)) *
                         ((wheel_base * M_PI) / (wheel_diameter * M_PI))) * 4.0) *
                       gear_ratio);
  right_speed.set__data((((vx * 60) / (wheel_diameter * M_PI)) -
                         (((60 * va) / (2 * M_PI)) *
                          ((wheel_base * M_PI) / (wheel_diameter * M_PI))) * 4.0) *
                        -gear_ratio);

  crawler_left_pub->publish(left_speed);
  crawler_right_pub->publish(right_speed);
}