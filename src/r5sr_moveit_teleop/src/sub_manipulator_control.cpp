#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;

namespace r5sr_moveit_teleop
{
class SubManipulatorControl : public rclcpp::Node
{
public:
  SubManipulatorControl(const rclcpp::NodeOptions & options)
  : Node("sub_manipulator_control", options)
  {
    pitch_correct = 0.0;
    pitch_root_command = 0.0;
    pitch_tip_command = 0.0;
    yaw_tip_command = 0.0;
    pitch_root_position = 0.1;
    pitch_tip_position = -M_PI_2;
    yaw_tip_position = 0.0;

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1, std::bind(&SubManipulatorControl::handle_joy, this, std::placeholders::_1));

    trajecory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/sub_arm_controller/joint_trajectory", 10);
    timer = this->create_wall_timer(50ms, std::bind(&SubManipulatorControl::timer_callback, this));
  }

  ~SubManipulatorControl() override {}

private:
  void handle_joy(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    pitch_root_command = joy->axes[12];

    const auto & buttons = joy->buttons;
    if (buttons[13]) {
      pitch_tip_command = 1.0;
    } else if (buttons[15]) {
      pitch_tip_command = -1.0;
    } else {
      pitch_tip_command = 0.0;
    }

    if (buttons[16]) {
      yaw_tip_command = 1.0;
    } else if (buttons[14]) {
      yaw_tip_command = -1.0;
    } else {
      yaw_tip_command = 0.0;
    }
  }

  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped body3_to_bodyv1_tf;
    try {
      body3_to_bodyv1_tf =
        tf_buffer->lookupTransform("vision_arm_body1_link", "body3_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform");
      return;
    }

    double roll_31, pitch_31, yaw_31;
    tf2::getEulerYPR(body3_to_bodyv1_tf.transform.rotation, yaw_31, pitch_31, roll_31);
    if (yaw_31 < -3.0) {
      pitch_31 = M_PI_2 + (M_PI_2 - pitch_31);
    }
    if (yaw_31 > 3.0) {
      pitch_31 = -M_PI_2 + (-M_PI_2 - pitch_31);
    }

    geometry_msgs::msg::TransformStamped base_to_vision1_tf;
    try {
      base_to_vision1_tf =
        tf_buffer->lookupTransform("vision_arm_body1_link", "body0_link_yaw", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform");
      return;
    }

    double yaw_v1, pitch_v1, roll_v1;
    tf2::getEulerYPR(base_to_vision1_tf.transform.rotation, yaw_v1, pitch_v1, roll_v1);
    if (yaw_v1 < -3.0) {
      pitch_v1 = M_PI_2 + (M_PI_2 - pitch_v1);
    }
    if (yaw_v1 > 3.0) {
      pitch_v1 = -M_PI_2 + (-M_PI_2 - pitch_v1);
    }

    const float speed_coef = 0.03;
    pitch_root_position += pitch_root_command * speed_coef;
    pitch_tip_position += pitch_tip_command * speed_coef;
    yaw_tip_position += yaw_tip_command * speed_coef;

    if (pitch_31 > 1.5) {
      pitch_correct += pitch_31 - 1.5;
    }

    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp.sec = 0;
    joint_trajectory.header.stamp.nanosec = 0;
    joint_trajectory.header.frame_id = "body4_link";

    joint_trajectory.joint_names.push_back("vision_arm_body1_joint");
    joint_trajectory.joint_names.push_back("vision_arm_body2_joint");
    joint_trajectory.joint_names.push_back("vision_arm_body3_joint");

    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point;
    joint_trajectory_point.positions.push_back(pitch_root_position + pitch_correct);
    joint_trajectory_point.positions.push_back(pitch_v1 + pitch_tip_position);
    joint_trajectory_point.positions.push_back(yaw_tip_position);
    joint_trajectory_point.time_from_start.sec = 0;
    joint_trajectory_point.time_from_start.nanosec = 0;

    joint_trajectory.points.push_back(joint_trajectory_point);

    trajecory_pub->publish(joint_trajectory);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajecory_pub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::TimerBase::SharedPtr timer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  float pitch_correct;
  float pitch_root_command;
  float pitch_tip_command;
  float yaw_tip_command;
  float pitch_root_position;
  float pitch_tip_position;
  float yaw_tip_position;

};  // class JoyToServo

}  // namespace r5sr_moveit_teleop

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(r5sr_moveit_teleop::SubManipulatorControl)
