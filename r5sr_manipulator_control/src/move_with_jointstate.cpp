#include "move_with_jointstate.hpp"

using namespace r5sr_manipulator_control;
using std::placeholders::_1;

MoveWithJointState::MoveWithJointState() : Node("MoveWithJointState") {
  this->declare_parameter("portname", "/dev/ttyUSB-Dynamixel");
  this->declare_parameter("baudrate", 1'000'000);

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

  joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1,
      std::bind(&MoveWithJointState::handle_joint_state, this, _1));
}

void MoveWithJointState::handle_joint_state(
    const sensor_msgs::msg::JointState::SharedPtr joint_state) {
  const auto size = joint_state->name.size();

  for (size_t i = 0; i < size; i++) {
    const std::string& name = joint_state->name.at(i);
    if (name == "body0_joint_yaw") continue;
    const auto& [model, id, offset_degree, coef] = jointname_id_model_map.at(name);
    const auto& [addr_torque_enable, addr_goal_position, pulse_per_rev] =
        model_addr_map.at(model);

    const float offset_rad = offset_degree * (M_PI/180.0);
    const auto target_rad = joint_state->position.at(i) * coef + offset_rad;
    const int32_t target_pulse = (int32_t)((int32_t)pulse_per_rev / (float)(2 * M_PI) * (float)target_rad);

    packetHandler->write4ByteTxRx(portHandler.get(), id, addr_goal_position,
                                  target_pulse);
  }
}