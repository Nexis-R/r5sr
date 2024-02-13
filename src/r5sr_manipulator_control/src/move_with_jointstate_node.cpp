#include "move_with_jointstate.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<r5sr_manipulator_control::MoveWithJointState>());
  rclcpp::shutdown();
}