#include "r5sr_crawler_control/r5sr_crawler_control.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crawler_control::Crawler_Control>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}