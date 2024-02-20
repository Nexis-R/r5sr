#include "r5sr_crawler_control/r5sr_crawler_control.hpp"

using namespace crawler_control;

using std::placeholders::_1;

Crawler_Control::Crawler_Control(const rclcpp::NodeOptions & options)
: Node("r5sr_crawler_control", options)
{
  this->declare_parameter("wheel_diameter", 1.0);
  this->declare_parameter("wheel_base", 1.0);
  this->declare_parameter("flipper_gear_ratio", 4500.0);
  this->declare_parameter("crawler_gear_ratio", 60.0);

  wheel_diameter = this->get_parameter("wheel_diameter").as_double();
  wheel_base = this->get_parameter("wheel_base").as_double();
  flipper_gear_ratio = this->get_parameter("flipper_gear_ratio").as_double();
  crawler_gear_ratio = this->get_parameter("crawler_gear_ratio").as_double();

  twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 1, std::bind(&Crawler_Control::handle_twist, this, _1));
  jointstate_sub = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 1, std::bind(&Crawler_Control::handle_jointstate, this, _1));

  crawler_left_pub = this->create_publisher<std_msgs::msg::Float32>("command_crawler_left", 1);
  crawler_right_pub = this->create_publisher<std_msgs::msg::Float32>("command_crawler_right", 1);

  flipper_left_front_pub =
    this->create_publisher<std_msgs::msg::Int32>("command_flipper_left_front", 1);
  flipper_right_front_pub =
    this->create_publisher<std_msgs::msg::Int32>("command_flipper_right_front", 1);
  flipper_left_back_pub =
    this->create_publisher<std_msgs::msg::Int32>("command_flipper_left_back", 1);
  flipper_right_back_pub =
    this->create_publisher<std_msgs::msg::Int32>("command_flipper_right_back", 1);
}

void Crawler_Control::handle_jointstate(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const auto counts_per_revolution = 500;
  const auto gear_ratio = 23.0 * 270;

  const auto rad_to_position = [counts_per_revolution, gear_ratio](const auto radians) {
    return radians * (counts_per_revolution * gear_ratio) / (2 * M_PI);
  };

  const auto find_position = [msg](const std::string joint_name) {
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
    if (it != msg->name.end()) {
      int index = std::distance(msg->name.begin(), it);
      return msg->position[index];
    }
    return 0.0;
  };

  std_msgs::msg::Int32 position;
  position.set__data(-1 * rad_to_position(find_position("flipper_left_front_joint")));
  flipper_left_front_pub->publish(position);

  position.set__data(-1 * rad_to_position(find_position("flipper_right_front_joint")));
  flipper_right_front_pub->publish(position);

  position.set__data(rad_to_position(find_position("flipper_left_rear_joint")));
  flipper_left_back_pub->publish(position);

  position.set__data(rad_to_position(find_position("flipper_right_rear_joint")));
  flipper_right_back_pub->publish(position);
}

void Crawler_Control::handle_twist(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  const float linear_velocity = twist->linear.x;
  const float angular_velocity = twist->angular.z;

  std_msgs::msg::Float32 left_wheel_speed;
  std_msgs::msg::Float32 right_wheel_speed;

  // Calculate the common term for linear velocity to RPM conversion
  float linear_velocity_to_rpm = (linear_velocity * 60) / (wheel_diameter * M_PI);

  // Calculate the common term for how the angular velocity affects the wheels
  float angular_effect_on_wheels =
    (((60 * angular_velocity) / (2 * M_PI)) * ((wheel_base * M_PI) / (wheel_diameter * M_PI))) *
    4.0;

  float left_wheel_speed_data =
    (linear_velocity_to_rpm + angular_effect_on_wheels) * crawler_gear_ratio;
  float right_wheel_speed_data =
    (linear_velocity_to_rpm - angular_effect_on_wheels) * -crawler_gear_ratio;

  left_wheel_speed.set__data(left_wheel_speed_data);
  right_wheel_speed.set__data(right_wheel_speed_data);

  crawler_left_pub->publish(left_wheel_speed);
  crawler_right_pub->publish(right_wheel_speed);
}  // namespace r5sr_crawler_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crawler_control::Crawler_Control)