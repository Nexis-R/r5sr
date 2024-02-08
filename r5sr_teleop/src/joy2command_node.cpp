#include "r5sr_teleop/joy2command.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<r5sr_teleop::Joy2Command>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}