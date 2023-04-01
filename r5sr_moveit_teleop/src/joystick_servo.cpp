#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const std::string FRAME_ID = "body5_link";

enum Axis {
  LEFT_STICK_X = 8,
  LEFT_STICK_Y = 9,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 10,
  RIGHT_STICK_Y = 11,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum Button {
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

std::map<Axis, double> AXIS_DEFAULTS = {{LEFT_TRIGGER, 1.0},
                                        {RIGHT_TRIGGER, 1.0}};
std::map<Button, double> BUTTON_DEFAULTS;

bool convertJoyToCmd(const std::vector<float>& axes,
                     const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint) {
  if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || buttons[15] ||
      buttons[16] || axes[D_PAD_X] || axes[D_PAD_Y]) {
    joint->joint_names.push_back("body1_joint");
    joint->velocities.push_back(axes[D_PAD_X]);
    joint->joint_names.push_back("body2_joint");
    joint->velocities.push_back(axes[D_PAD_Y]);

    joint->joint_names.push_back("body6_joint");
    joint->velocities.push_back((buttons[Y] - buttons[B]) * 1.5);
    joint->joint_names.push_back("body5_joint");
    joint->velocities.push_back(buttons[17] - buttons[18]);
    return false;
  }

  twist->twist.linear.z = axes[RIGHT_STICK_Y];
  twist->twist.linear.y = axes[RIGHT_STICK_X];

  double lin_x_right =
      -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
  double lin_x_left =
      0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
  twist->twist.linear.x = lin_x_right + lin_x_left;

  twist->twist.angular.y = axes[LEFT_STICK_Y];
  twist->twist.angular.x = axes[LEFT_STICK_X];

  double roll_positive = 1.5 * buttons[LEFT_BUMPER];
  double roll_negative = -1.5 * (buttons[RIGHT_BUMPER]);
  twist->twist.angular.z = roll_positive + roll_negative;

  return true;
}
namespace r5sr_moveit_teleop {
class JoyToServo : public rclcpp::Node {
 public:
  JoyToServo(const rclcpp::NodeOptions& options)
      : Node("joy_to_twist_publisher", options),
        frame_to_publish_(FRAME_ID) {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, ROS_QUEUE_SIZE,
        std::bind(&JoyToServo::joyCB, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        TWIST_TOPIC, ROS_QUEUE_SIZE);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
        JOINT_TOPIC, ROS_QUEUE_SIZE);
    collision_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
        "/planning_scene", 10);

    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  ~JoyToServo() override {}

  void joyCB(const sensor_msgs::msg::Joy::SharedPtr msg) {
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg)) {
      // publish the TwistStamped
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    } else {
      // publish the JointJog
      joint_msg->header.stamp = this->now();
      joint_msg->header.frame_id = "base_link";
      joint_pub_->publish(std::move(joint_msg));
    }
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  std::string frame_to_publish_;

};  // class JoyToServo

}  // namespace r5sr_moveit_teleop

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(r5sr_moveit_teleop::JoyToServo)
