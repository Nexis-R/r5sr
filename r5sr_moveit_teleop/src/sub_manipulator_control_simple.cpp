#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;

namespace r5sr_moveit_teleop {
class SubManipulatorSimpleControl : public rclcpp::Node {
 public:
  SubManipulatorSimpleControl(const rclcpp::NodeOptions& options)
      : Node("sub_manipulator_control", options) {
    pitch_correct = 0.0;
    pitch_root_command = 0.0;
    pitch_tip_command = 0.0;
    yaw_tip_command = 0.0;
    pitch_root_position = 0.1;
    pitch_tip_position = -0.19;
    yaw_tip_position = 0.0;

    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1,
        std::bind(&SubManipulatorSimpleControl::handle_joy, this,
                  std::placeholders::_1));

    trajecory_pub =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/overhead_arm_controller/joint_trajectory", 10);
    timer = this->create_wall_timer(
        50ms, std::bind(&SubManipulatorSimpleControl::timer_callback, this));
  }

  ~SubManipulatorSimpleControl() override {}

 private:
  void handle_joy(const sensor_msgs::msg::Joy::SharedPtr joy) {
    pitch_root_command = joy->axes[12];

    const auto& buttons = joy->buttons;
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

  void timer_callback() {
    const float speed_coef = 0.03;
    pitch_root_position += pitch_root_command * speed_coef;
    pitch_tip_position += pitch_tip_command * speed_coef;
    yaw_tip_position += yaw_tip_command * speed_coef;

    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp.sec = 0;
    joint_trajectory.header.stamp.nanosec = 0;
    joint_trajectory.header.frame_id = "wrist_1_link";

    joint_trajectory.joint_names.push_back("overhead_elbow_joint");
    joint_trajectory.joint_names.push_back("overhead_wrist_yaw_joint");
    joint_trajectory.joint_names.push_back("overhead_wrist_pitch_joint");

    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point;
    joint_trajectory_point.positions.push_back(pitch_root_position);
    joint_trajectory_point.positions.push_back(pitch_tip_position);
    joint_trajectory_point.positions.push_back(yaw_tip_position);
    joint_trajectory_point.time_from_start.sec = 0;
    joint_trajectory_point.time_from_start.nanosec = 0;

    joint_trajectory.points.push_back(joint_trajectory_point);

    trajecory_pub->publish(joint_trajectory);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajecory_pub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::TimerBase::SharedPtr timer;
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
RCLCPP_COMPONENTS_REGISTER_NODE(r5sr_moveit_teleop::SubManipulatorSimpleControl)
