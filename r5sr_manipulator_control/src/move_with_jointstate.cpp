#include "move_with_jointstate.hpp"

#include <array>

using namespace r5sr_manipulator_control;
using std::placeholders::_1;

MoveWithJointState::MoveWithJointState()
    : Node("MoveWithJointState"),
      hand_pulse(-1000),
      vision_angle(
          {std::get<2>(jointname_id_model_map.at("vision_arm_body1_joint")),
           std::get<2>(jointname_id_model_map.at("vision_arm_body2_joint")),
           std::get<2>(jointname_id_model_map.at("vision_arm_body3_joint"))}) {
  this->declare_parameter("portname", "/dev/ttyUSB-Dynamixel");
  this->declare_parameter("baudrate", 1'000'000);
  this->declare_parameter("open_button_index", 1);
  this->declare_parameter("close_button_index", 2);

  portHandler.reset(std::move(dynamixel::PortHandler::getPortHandler(
      this->get_parameter("portname").as_string().c_str())));
  packetHandler.reset(std::move(dynamixel::PacketHandler::getPacketHandler()));

  if (portHandler->openPort()) {
    RCLCPP_INFO(this->get_logger(), "Success to open port!");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open port!");
  }

  if (portHandler->setBaudRate(this->get_parameter("baudrate").as_int())) {
    RCLCPP_INFO(this->get_logger(), "Success to set baudrate!");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate!");
  }

  for (const auto& id_map : jointname_id_model_map) {
    const auto& joint_name = id_map.first;
    const auto& [model, id, offset, coef] = id_map.second;
    const auto& addr_torque_enable = std::get<0>(model_addr_map.at(model));

    packetHandler->write1ByteTxOnly(portHandler.get(), id, addr_torque_enable,
                                    1);
    const auto result = packetHandler->ping(portHandler.get(), id);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "id: " << (int)id << ",  joint: " << joint_name
                              << ",  ping result: " << result);
  }

  hand_current_pub =
      this->create_publisher<std_msgs::msg::Float32>("hand_current", 1);

  joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1,
      std::bind(&MoveWithJointState::handle_joint_state, this, _1));

  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1, std::bind(&MoveWithJointState::handle_joy, this, _1));
}

void MoveWithJointState::handle_joint_state(
    const sensor_msgs::msg::JointState::SharedPtr joint_state) {
  const auto size = joint_state->name.size();

  for (size_t i = 0; i < size; i++) {
    const std::string& name = joint_state->name.at(i);
    if (name == "body0_joint_yaw") continue;
    const auto& [model, id, offset_degree, coef] =
        jointname_id_model_map.at(name);
    const auto& [addr_torque_enable, addr_goal_position, pulse_per_rev] =
        model_addr_map.at(model);

    const float offset_rad = offset_degree * (M_PI / 180.0);
    const auto target_rad = joint_state->position.at(i) * coef + offset_rad;
    const int32_t target_pulse =
        (int32_t)((int32_t)pulse_per_rev / (float)(2 * M_PI) *
                  (float)target_rad);

    packetHandler->write4ByteTxRx(portHandler.get(), id, addr_goal_position,
                                  target_pulse);
  }
}

void MoveWithJointState::handle_joy(
    const sensor_msgs::msg::Joy::SharedPtr joy) {
  const auto& buttons = joy->buttons;
  const auto& axes = joy->axes;
  // hand
  const int open_button_index =
      this->get_parameter("open_button_index").as_int();
  const int close_button_index =
      this->get_parameter("close_button_index").as_int();

  const int hand_id = 8;

  const int pulse_step = 200;
  const int min_limit = -3000;
  const int max_limit = 3000;
  const float current_limit = 300.0;  // mA

  uint16_t current_raw;
  packetHandler->read2ByteTxRx(portHandler.get(), hand_id, 126, &current_raw);
  const float current = (int16_t)current_raw * 1.34;  // mA

  std_msgs::msg::Float32 current_pub;
  current_pub.data = current;
  hand_current_pub->publish(current_pub);

  if (buttons[open_button_index] && pulse_step > min_limit) {
    hand_pulse -= pulse_step;
  } else if (buttons[close_button_index] && pulse_step < max_limit &&
             current < current_limit) {
    hand_pulse += pulse_step;
  }

  packetHandler->write4ByteTxRx(portHandler.get(), hand_id,
                                std::get<1>(model_addr_map.at(XH430)),
                                hand_pulse);
}