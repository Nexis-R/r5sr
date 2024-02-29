#include "r5sr_cloud/rms_robot.hpp"

namespace r5sr_cloud
{
RmsRobot::RmsRobot(const rclcpp::NodeOptions & options) : Node("rms_robot", options) {}
}  // namespace r5sr_cloud

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(r5sr_cloud::RmsRobot)