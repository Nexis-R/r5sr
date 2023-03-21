#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace r5sr_moveit_teleop {
class PresetPose : public rclcpp::Node {
 public:
  PresetPose(const rclcpp::NodeOptions& options)
      : Node("preset_pose", options) {
    joint_trajectory_pub =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/main_arm_controller/joint_trajectory", 10);
    stop_servo_client =
        this->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
    start_servo_client =
        this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");

    // wait for service
    while (!stop_servo_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Waiting for stop servo service");
    }
    while (!start_servo_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Waiting for start servo service");
    }

    move_to_default_pose_service = this->create_service<std_srvs::srv::Empty>(
        "~/move_to_default_pose",
        std::bind(&PresetPose::moveToDefaultPoseCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    move_to_floor_pose_service = this->create_service<std_srvs::srv::Empty>(
        "~/move_to_floor_pose",
        std::bind(&PresetPose::moveToFloorPoseCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

 private:
  void moveToPositions(const std::vector<double>& positions) {
    // stop servo
    auto stop_servo_future = stop_servo_client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO_STREAM(this->get_logger(), "call stop servo");

    // delay
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // make joint trajectory
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp.sec = 0;
    joint_trajectory.header.stamp.nanosec = 0;
    joint_trajectory.header.frame_id = "base_link";

    joint_trajectory.joint_names = {"body1_joint", "body2_joint",
                                    "body3_joint", "body4_joint",
                                    "body5_joint", "body6_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point;
    joint_trajectory_point.time_from_start.sec = 4;
    joint_trajectory_point.time_from_start.nanosec = 0;
    joint_trajectory_point.positions = positions;

    joint_trajectory.points.push_back(joint_trajectory_point);

    joint_trajectory_pub->publish(joint_trajectory);
    RCLCPP_INFO_STREAM(this->get_logger(), "publish joint trajectory");

    // delay
    rclcpp::sleep_for(std::chrono::milliseconds(4000));

    // start servo
    auto start_servo_future = start_servo_client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO_STREAM(this->get_logger(), "call start servo");
  }

  void moveToDefaultPoseCallback(
      const std_srvs::srv::Empty::Request::SharedPtr request,
      const std_srvs::srv::Empty::Response::SharedPtr response) {
    moveToPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  }
  void moveToFloorPoseCallback(
      const std_srvs::srv::Empty::Request::SharedPtr request,
      const std_srvs::srv::Empty::Response::SharedPtr response) {
    moveToPositions({-0.005899328800385288, 0.7083160049999053,
                     -0.6195460128516843, 1.4261574842731133,
                     -0.0003898799515165366, -0.005937718504569254});
  }

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_to_default_pose_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_to_floor_pose_service;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      joint_trajectory_pub;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client;
};
}  // namespace r5sr_moveit_teleop

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(r5sr_moveit_teleop::PresetPose)