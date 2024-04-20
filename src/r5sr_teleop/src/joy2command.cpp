#include <r5sr_interface/msg/crawler_mode.hpp>
#include <r5sr_interface/msg/crawler_speed.hpp>
#include <r5sr_interface/msg/manipulator_speed.hpp>
#include <r5sr_interface/msg/teleop_mode.hpp>
#include <r5sr_teleop/joy2command.hpp>
#include <rclcpp/logging.hpp>

namespace r5sr_teleop
{
Joy2Command::Joy2Command(const rclcpp::NodeOptions & options) : Node("joy2command", options)
{
  twist_crawler_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  twist_arm_publisher =
    this->create_publisher<geometry_msgs::msg::TwistStamped>("delta_twist_cmds", 10);
  joint_arm_publisher = this->create_publisher<control_msgs::msg::JointJog>("joint_jog", 10);
  joint_overhead_arm_publisher =
    this->create_publisher<control_msgs::msg::JointJog>("joint_jog_overhead", 10);
  joint_flippers_publisher =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>("flipper_joint_trajectory", 10);
  hand_publisher = this->create_publisher<std_msgs::msg::Float32>("hand", 10);
  mode_change_publisher =
    this->create_publisher<r5sr_interface::msg::OperationMode>("mode_change", 10);
  joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&Joy2Command::handle_joy, this, std::placeholders::_1));
  mode_change_subscription = this->create_subscription<r5sr_interface::msg::OperationMode>(
    "mode_change", 10, std::bind(&Joy2Command::handle_change_mode, this, std::placeholders::_1));

  emergency_status_subscription = this->create_subscription<std_msgs::msg::Bool>(
    "is_emergency_stopped", 10, std::bind(&Joy2Command::handle_emergency_status, this, std::placeholders::_1));

  servo_start_client = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  servo_start_client->wait_for_service(std::chrono::seconds(1));
  servo_start_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

  overhead_servo_start_client =
    this->create_client<std_srvs::srv::Trigger>("/overhead_servo_node/start_servo");
  overhead_servo_start_client->wait_for_service(std::chrono::seconds(1));
  overhead_servo_start_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());

  operation_mode.teleop_mode = r5sr_interface::msg::TeleopMode::CRAWLER;
  operation_mode.crawler_mode = r5sr_interface::msg::CrawlerMode::FORWARD;
  operation_mode.crawler_speed = r5sr_interface::msg::CrawlerSpeed::NORMAL;
  operation_mode.manipulator_speed = r5sr_interface::msg::ManipulatorSpeed::NORMAL;
}

void Joy2Command::handle_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (last_joy_msg.buttons.empty()) {
    last_joy_msg = *msg;
    return;
  }

  const auto & buttons = msg->buttons;
  const auto & axes = msg->axes;

  // operation mode
  if (emergency_status_msg.data != true) {
    if (buttons[Button::LEFT_STICK]) {
      auto operation_mode_msg = operation_mode;
      operation_mode_msg.teleop_mode = r5sr_interface::msg::TeleopMode::CRAWLER;

      mode_change_publisher->publish(operation_mode_msg);
    }
    if (buttons[Button::RIGHT_STICK]) {
      auto operation_mode_msg = operation_mode;
      operation_mode_msg.teleop_mode = r5sr_interface::msg::TeleopMode::MANIPULATOR;

      mode_change_publisher->publish(operation_mode_msg);
    }
  }else{
      auto operation_mode_msg = operation_mode;
      operation_mode_msg.teleop_mode = r5sr_interface::msg::TeleopMode::STOP;

      mode_change_publisher->publish(operation_mode_msg);

  }

  switch (operation_mode.teleop_mode) {
    case r5sr_interface::msg::TeleopMode::CRAWLER:

      // crawler mode change
      if (buttons[Button::OPTIONS]) {
        auto operation_mode_msg = operation_mode;
        operation_mode_msg.crawler_mode = r5sr_interface::msg::CrawlerMode::FORWARD;

        mode_change_publisher->publish(operation_mode_msg);
      }
      if (buttons[Button::SHARE]) {
        auto operation_mode_msg = operation_mode;
        operation_mode_msg.crawler_mode = r5sr_interface::msg::CrawlerMode::BACKWARD;

        mode_change_publisher->publish(operation_mode_msg);
      }
      if (buttons[Button::PS] && !last_joy_msg.buttons[Button::PS]) {
        auto operation_mode_msg = operation_mode;
        auto crawler_speed = operation_mode.crawler_speed;
        ++crawler_speed;
        if (crawler_speed > 3) {
          crawler_speed = 0;
        }
        operation_mode_msg.crawler_speed = crawler_speed;

        mode_change_publisher->publish(operation_mode_msg);
      }

      // crawler
      {
        const float scale = [](const auto & operation_mode) {
          switch (operation_mode.crawler_speed) {
            case r5sr_interface::msg::CrawlerSpeed::NORMAL:
            default:
              return 0.8;
            case r5sr_interface::msg::CrawlerSpeed::FAST:
              return 1.0;
            case r5sr_interface::msg::CrawlerSpeed::SLOW:
              return 0.5;
            case r5sr_interface::msg::CrawlerSpeed::SUPER_SLOW:
              return 0.2;
          }
        }(operation_mode);
        geometry_msgs::msg::Twist twist_msg;
        const auto linear_x = axes[Axis::LEFT_STICK_Y] * scale;
        twist_msg.linear.x =
          (operation_mode.crawler_mode == r5sr_interface::msg::CrawlerMode::FORWARD) ? linear_x
                                                                                     : -linear_x;
        twist_msg.angular.z = -axes[Axis::RIGHT_STICK_X] * scale;

        twist_crawler_publisher->publish(twist_msg);
      }

      // flippers
      {
        auto flipper_speed = [](const bool button, const float stick) -> float {
          return button ? stick * 0.0050 : 0.0;
        };

        this->flippers_position = {
          flippers_position[0] -
            flipper_speed(buttons[Button::LEFT_BUMPER], axes[Axis::RIGHT_STICK_Y]),
          flippers_position[1] +
            flipper_speed(buttons[Button::LEFT_TRIGGER_BUTTON], axes[Axis::RIGHT_STICK_Y]),
          flippers_position[2] +
            flipper_speed(buttons[Button::RIGHT_TRIGGER_BUTTON], axes[Axis::RIGHT_STICK_Y]),
          flippers_position[3] -
            flipper_speed(buttons[Button::RIGHT_BUMPER], axes[Axis::RIGHT_STICK_Y]),
        };

        trajectory_msgs::msg::JointTrajectory flippers_msg;
        flippers_msg.joint_names = {
          "flipper_left_front_joint", "flipper_left_rear_joint", "flipper_right_rear_joint",
          "flipper_right_front_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start = rclcpp::Duration(0, 0);
        for (const auto & pos : flippers_position) {
          point.positions.push_back(pos);
        }
        flippers_msg.points.push_back(point);

        joint_flippers_publisher->publish(flippers_msg);
      }

      break;

    case r5sr_interface::msg::TeleopMode::MANIPULATOR:

      // manipulator mode change
      if (buttons[Button::PS] && !last_joy_msg.buttons[Button::PS]) {
        auto operation_mode_msg = operation_mode;
        auto manipulator_speed = operation_mode.manipulator_speed;
        ++manipulator_speed;
        if (manipulator_speed > 3) {
          manipulator_speed = 0;
        }
        operation_mode_msg.manipulator_speed = manipulator_speed;

        mode_change_publisher->publish(operation_mode_msg);
      }

      // overhead_arm
      if (buttons[Button::OPTIONS] || buttons[Button::SHARE]) {
        control_msgs::msg::JointJog joint_msg;
        joint_msg.joint_names.push_back("overhead_elbow_joint");
        joint_msg.joint_names.push_back("overhead_wrist_pitch_joint");
        joint_msg.joint_names.push_back("overhead_wrist_yaw_joint");
        joint_msg.velocities.push_back(axes[DPAD_Y] * 2.0);
        if (buttons[Button::SQUARE]) {
          joint_msg.velocities.push_back(2.0);
        } else if (buttons[Button::CIRCLE]) {
          joint_msg.velocities.push_back(-2.0);
        } else {
          joint_msg.velocities.push_back(0.0);
        }
        if (buttons[Button::TRIANGLE]) {
          joint_msg.velocities.push_back(2.0);
        } else if (buttons[Button::CROSS]) {
          joint_msg.velocities.push_back(-2.0);
        } else {
          joint_msg.velocities.push_back(0.0);
        }
        joint_msg.header.stamp = this->now();
        joint_msg.header.frame_id = "wrist_1_link";
        joint_overhead_arm_publisher->publish(joint_msg);
      } else {
        if (axes[DPAD_X] || axes[DPAD_Y] || buttons[Button::CIRCLE] || buttons[Button::SQUARE]) {
          // joint jog
          control_msgs::msg::JointJog joint_msg;
          joint_msg.joint_names.push_back("body_joint");
          joint_msg.velocities.push_back(axes[DPAD_X]);
          joint_msg.joint_names.push_back("shoulder_joint");
          joint_msg.velocities.push_back(axes[DPAD_Y]);

          joint_msg.joint_names.push_back("wrist_roll_joint");
          joint_msg.velocities.push_back(buttons[Button::SQUARE] - buttons[Button::CIRCLE]);

          joint_msg.header.stamp = this->now();
          joint_msg.header.frame_id = "body_link";
          joint_arm_publisher->publish(joint_msg);
        } else {
          // arm twist
          geometry_msgs::msg::TwistStamped twist_msg;

          const auto scale = [](const auto & operation_mode) -> float {
            switch (operation_mode.manipulator_speed) {
              case r5sr_interface::msg::ManipulatorSpeed::NORMAL:
              default:
                return 0.8;
              case r5sr_interface::msg::ManipulatorSpeed::FAST:
                return 1.0;
              case r5sr_interface::msg::ManipulatorSpeed::SLOW:
                return 0.5;
              case r5sr_interface::msg::ManipulatorSpeed::SUPER_SLOW:
                return 0.2;
            }
          }(operation_mode);

          twist_msg.twist.linear.z = axes[RIGHT_STICK_Y] * scale;
          twist_msg.twist.linear.y = axes[RIGHT_STICK_X] * scale;

          const auto linear_x_right = -0.5 * (axes[RIGHT_TRIGGER] - 1.0) * scale;
          const auto linear_x_left = 0.5 * (axes[LEFT_TRIGGER] - 1.0) * scale;
          twist_msg.twist.linear.x = linear_x_right + linear_x_left;

          twist_msg.twist.angular.y = axes[LEFT_STICK_Y] * scale;
          twist_msg.twist.angular.x = axes[LEFT_STICK_X] * scale;

          const auto roll_positive = buttons[LEFT_BUMPER] * scale;
          const auto roll_negative = -1 * (buttons[RIGHT_BUMPER]) * scale;
          twist_msg.twist.angular.z = roll_positive + roll_negative;

          twist_msg.header.stamp = this->now();
          twist_msg.header.frame_id = "wrist_2_link";
          twist_arm_publisher->publish(twist_msg);
        }

        // hand
        {
          std_msgs::msg::Float32 hand_msg;
          hand_msg.data = buttons[Button::TRIANGLE] - buttons[Button::CROSS];

          hand_publisher->publish(hand_msg);
        }

        {
          geometry_msgs::msg::Twist twist_msg;
          twist_crawler_publisher->publish(twist_msg);
        }
      }
      break;
    case r5sr_interface::msg::TeleopMode::STOP:
    default:
      break;
  }

  last_joy_msg = *msg;
}

void Joy2Command::handle_change_mode(const r5sr_interface::msg::OperationMode::SharedPtr msg)
{
  operation_mode = *msg;
}


void Joy2Command::handle_emergency_status(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_status_msg = *msg;
}


}  // namespace r5sr_teleop

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(r5sr_teleop::Joy2Command)