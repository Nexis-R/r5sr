#ifndef R5SR_TELEROP_JOY2COMMAND_HPP_
#define R5SR_TELEROP_JOY2COMMAND_HPP_

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <r5sr_interface/msg/operation_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace r5sr_teleop
{

enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  DPAD_X = 6,
  DPAD_Y = 7,
};

enum Button
{
  CROSS = 0,
  CIRCLE = 1,
  TRIANGLE = 2,
  SQUARE = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  LEFT_TRIGGER_BUTTON = 6,
  RIGHT_TRIGGER_BUTTON = 7,
  SHARE = 8,
  OPTIONS = 9,
  PS = 10,
  LEFT_STICK = 11,
  RIGHT_STICK = 12
};

class Joy2Command : public rclcpp::Node
{
public:
  explicit Joy2Command(const rclcpp::NodeOptions& options);

private:
  void handle_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void handle_change_mode(const r5sr_interface::msg::OperationMode::SharedPtr msg);

  r5sr_interface::msg::OperationMode operation_mode;
  sensor_msgs::msg::Joy last_joy_msg;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate_subscription;
  rclcpp::Subscription<r5sr_interface::msg::OperationMode>::SharedPtr mode_change_subscription;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_crawler_publisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_arm_publisher;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_arm_publisher;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_overhead_arm_publisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_flippers_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hand_publisher;
  rclcpp::Publisher<r5sr_interface::msg::OperationMode>::SharedPtr mode_change_publisher;
  rclcpp::Publisher<r5sr_interface::msg::OperationMode>::SharedPtr operation_mode_publisher;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr overhead_servo_start_client;

  std::array<float, 4> flippers_position;
};

}  // namespace r5sr_teleop

#endif  // R5SR_TELEROP_JOY2COMMAND_HPP_