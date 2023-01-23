#include "r5sr_teleop/remap_joy.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<r5sr_teleop::RemapJoy>());
  rclcpp::shutdown();
  return 0;
}