#include "move_with_jointstate.hpp"

#include <array>

using namespace r5sr_manipulator_control;
using std::placeholders::_1;

MoveWithJointState::MoveWithJointState()
: Node("MoveWithJointState"),
  hand_pulse(1000),
  vision_angle(
    {std::get<2>(jointname_id_model_map.at("overhead_elbow_joint")),
     std::get<2>(jointname_id_model_map.at("overhead_wrist_yaw_joint")),
     std::get<2>(jointname_id_model_map.at("overhead_wrist_pitch_joint"))})
{
  this->declare_parameter("portname", "/dev/ttyUSB-Dynamixel");
  this->declare_parameter("baudrate", 1'000'000);

  portHandler.reset(std::move(
    dynamixel::PortHandler::getPortHandler(this->get_parameter("portname").as_string().c_str())));
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

  for (const auto & id_map : jointname_id_model_map) {
    const auto & joint_name = id_map.first;
    const auto & [model, id, offset, coef] = id_map.second;
    const auto & addr_torque_enable = std::get<0>(model_addr_map.at(model));

    packetHandler->write1ByteTxOnly(portHandler.get(), id, addr_torque_enable, 1);
    const auto result = packetHandler->ping(portHandler.get(), id);
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "id: " << (int)id << ",  joint: " << joint_name << ",  ping result: " << result);
  }

  voltage_pub = this->create_publisher<std_msgs::msg::Float32>("~/voltage", 1);
  hand_current_pub = this->create_publisher<std_msgs::msg::Float32>("~/hand_current", 1);

  joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 1, std::bind(&MoveWithJointState::handle_joint_state, this, _1));
  hand_command_sub = this->create_subscription<std_msgs::msg::Float32>(
    "/hand", 1, std::bind(&MoveWithJointState::handle_hand_command, this, _1));
}

void MoveWithJointState::handle_joint_state(
  const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
  const auto size = joint_state->name.size();

  for (size_t i = 0; i < size; i++) {
    const std::string & name = joint_state->name.at(i);
    const auto it = jointname_id_model_map.find(name);
    if (it == jointname_id_model_map.end()) {
      continue;
    }
    const auto & [model, id, offset_degree, coef] = jointname_id_model_map.at(name);
    const auto & [addr_torque_enable, addr_goal_position, pulse_per_rev] = model_addr_map.at(model);

    const float offset_rad = offset_degree * (M_PI / 180.0);
    const auto target_rad = joint_state->position.at(i) * coef + offset_rad;
    const int32_t target_pulse =
      (int32_t)((int32_t)pulse_per_rev / (float)(2 * M_PI) * (float)target_rad);

    packetHandler->write4ByteTxRx(portHandler.get(), id, addr_goal_position, target_pulse);
  }
}

void MoveWithJointState::handle_hand_command(const std_msgs::msg::Float32::SharedPtr command)
{
  const int hand_id = 8;

  const int pulse_step = 400;
  const int min_limit = -3000;
  const int max_limit = 3000;
  const float current_limit = 300.0;  // mA

  uint16_t voltage_raw;
  packetHandler->read2ByteTxRx(portHandler.get(), hand_id, 144, &voltage_raw);
  const float voltage = voltage_raw * 0.1;  // V

  std_msgs::msg::Float32 voltage_msg;
  voltage_msg.data = voltage;
  voltage_pub->publish(voltage_msg);

  uint16_t current_raw;
  packetHandler->read2ByteTxRx(portHandler.get(), hand_id, 126, &current_raw);
  const float current = (int16_t)current_raw * 1.34;  // mA

  std_msgs::msg::Float32 current_pub;
  current_pub.data = current;
  hand_current_pub->publish(current_pub);

  const float com = command->data;
  if (com < -0.1 && pulse_step > min_limit) {
    hand_pulse -= pulse_step;
  } else if (com > 0.1 && pulse_step < max_limit && current < current_limit) {
    hand_pulse += pulse_step;
  }

  packetHandler->write4ByteTxRx(
    portHandler.get(), hand_id, std::get<1>(model_addr_map.at(XH430)), hand_pulse);
}