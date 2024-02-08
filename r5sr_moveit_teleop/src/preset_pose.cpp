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
            "/arm_controller/joint_trajectory", 10);
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
    move_to_high_pose_service = this->create_service<std_srvs::srv::Empty>(
        "~/move_to_high_pose",
        std::bind(&PresetPose::moveToHighPoseCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    move_to_dex1_center_pose_service = this->create_service<std_srvs::srv::Empty>(
        "preset_pose_node/move_to_dex1_center_pose",
        std::bind(&PresetPose::moveToDex1CenterPoseCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    move_to_dex1_diaright_pose_service = this->create_service<std_srvs::srv::Empty>(
        "preset_pose_node/move_to_dex1_diaright_pose",
        std::bind(&PresetPose::moveToDex1DiaRightPoseCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    move_to_dex1_dialeft_pose_service = this->create_service<std_srvs::srv::Empty>(
        "preset_pose_node/move_to_dex1_dialeft_pose",
        std::bind(&PresetPose::moveToDex1DiaLeftPoseCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    move_to_dex1_right_pose_service = this->create_service<std_srvs::srv::Empty>(
        "preset_pose_node/move_to_dex1_right_pose",
        std::bind(&PresetPose::moveToDex1RightPoseCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    move_to_dex1_left_pose_service = this->create_service<std_srvs::srv::Empty>(
        "preset_pose_node/move_to_dex1_left_pose",
        std::bind(&PresetPose::moveToDex1LeftPoseCallback, this,
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
    joint_trajectory.header.frame_id = "body_link";

    joint_trajectory.joint_names = {"body_joint", "shoulder_joint",
                                    "elbow_joint", "forearm_joint",
                                    "wrist_yaw_joint", "wrist_roll_joint"};

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
  void moveToHighPoseCallback(
      const std_srvs::srv::Empty::Request::SharedPtr request,
      const std_srvs::srv::Empty::Response::SharedPtr response) {
    moveToPositions({-2.972387139899503e-05, 0.38043264539814864,
                     -0.6489122199681415, 0.27611256317117366,
                     -1.96441695861549e-06, -2.9917298595345435e-05});
  }

  void moveToDex1CenterPoseCallback(
      const std_srvs::srv::Empty::Request::SharedPtr request,
      const std_srvs::srv::Empty::Response::SharedPtr response) {
    moveToPositions({0.01991229029153637, 1.6294818377236404,
                     -1.0059121381250555, 0.8913840174821814,
                     -0.0018309824484326255, 0.01983365511952526});
  }
  void moveToDex1DiaRightPoseCallback(
      const std_srvs::srv::Empty::Request::SharedPtr request,
      const std_srvs::srv::Empty::Response::SharedPtr response) {
    //ドライバが器具に当たらないように回避ポイントを経由する
    moveToPositions({0.014407121838988318, 1.5085494110096298,
                     -1.0110233745048391, 1.0174188269794784,
                     -0.0015236838048826179, 0.014337062917931768});

    moveToPositions({-0.5557149518823368, 1.896163636947308,
                     -1.1273298882506417, 1.0606426782545288,
                     0.8041097459034522, -0.6749136136530615});
  }
  void moveToDex1DiaLeftPoseCallback(
      const std_srvs::srv::Empty::Request::SharedPtr request,
      const std_srvs::srv::Empty::Response::SharedPtr response) {
    //ドライバが器具に当たらないように回避ポイントを経由する
    moveToPositions({0.014407121838988318, 1.5085494110096298,
                     -1.0110233745048391, 1.0174188269794784,
                     -0.0015236838048826179, 0.014337062917931768});
                     
    moveToPositions({0.5341102027497134, 1.9730644846282765,
                     -1.191104160197627, 1.151873752530019,
                     -0.7118592461765382, 0.6039940316247959});
  }

  void moveToDex1RightPoseCallback(
      const std_srvs::srv::Empty::Request::SharedPtr request,
      const std_srvs::srv::Empty::Response::SharedPtr response){
    //ドライバが器具に当たらないように回避ポイントを経由する
    moveToPositions({0.014407121838988318, 1.5085494110096298,
                     -1.0110233745048391, 1.0174188269794784,
                     -0.0015236838048826179, 0.014337062917931768});

    moveToPositions({-0.45375902007816393, 1.7886923464987132,
                    -1.168202155866634, 0.899908021980262,
                    0.023765920900536426, -0.45318708630271093});
  }

  void moveToDex1LeftPoseCallback(
      const std_srvs::srv::Empty::Request::SharedPtr request,
      const std_srvs::srv::Empty::Response::SharedPtr response){
    //ドライバが器具に当たらないように回避ポイントを経由する
    moveToPositions({0.014407121838988318, 1.5085494110096298,
                     -1.0110233745048391, 1.0174188269794784,
                     -0.0015236838048826179, 0.014337062917931768});
                     
    moveToPositions({0.5059943281768762, 1.804458871384405,
                    -1.1532734772612905, 0.8711237825894158,
                    -0.02767158449195166, 0.5052967924862063});
  }


  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_to_default_pose_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_to_floor_pose_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_to_high_pose_service;
  
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_to_dex1_center_pose_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_to_dex1_diaright_pose_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_to_dex1_dialeft_pose_service;
  
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_to_dex1_left_pose_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_to_dex1_right_pose_service;
  
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      joint_trajectory_pub;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client;
};
}  // namespace r5sr_moveit_teleop

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(r5sr_moveit_teleop::PresetPose)