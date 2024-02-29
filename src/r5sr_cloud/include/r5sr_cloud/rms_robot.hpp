#ifndef R5SR_CLOUD__RMS_ROBOT_HPP_
#define R5SR_CLOUD__RMS_ROBOT_HPP_

#include "rclcpp/rclcpp.hpp"

namespace r5sr_cloud
{
class RmsRobot : public rclcpp::Node
{
public:
  explicit RmsRobot(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};
}  // namespace r5sr_cloud

#endif