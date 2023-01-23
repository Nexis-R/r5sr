#include "r5sr_teleop/remap_joy.hpp"

using namespace r5sr_teleop;

RemapJoy::RemapJoy()
    : Node("teleop_state_broadcaster"), teleop_mode("crawler") {
  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1, std::bind(&RemapJoy::handle_joy, this, std::placeholders::_1));

  teleop_mode_pub =
      this->create_publisher<std_msgs::msg::String>("teleop_mode", 1);
  joy_pub = this->create_publisher<sensor_msgs::msg::Joy>("out", 1);
}

void RemapJoy::handle_joy(const sensor_msgs::msg::Joy::SharedPtr joy) {
  const auto& buttons = joy->buttons;

  if (buttons[9]) {
    teleop_mode = "crawler";
  } else if (buttons[10]) {
    teleop_mode = "manipulator";
  }

  std_msgs::msg::String mode_msg;
  mode_msg.data = teleop_mode;

  sensor_msgs::msg::Joy joy_repub = *joy.get();

  auto& axes_repub = joy_repub.axes;
  auto& buttons_repub = joy_repub.buttons;
  if (teleop_mode == "crawler") {
    // add manipulator mode stick (not move)
    axes_repub.push_back(0.0);
    axes_repub.push_back(0.0);
    axes_repub.push_back(0.0);
    axes_repub.push_back(0.0);
  } else if (teleop_mode == "manipulator") {
    // add manipulator mode stick
    axes_repub.push_back(axes_repub[0]);
    axes_repub.push_back(axes_repub[1]);
    axes_repub.push_back(axes_repub[3]);
    axes_repub.push_back(axes_repub[4]);
    // remove crawler move
    axes_repub[0] = 0.0;
    axes_repub[1] = 0.0;
    axes_repub[3] = 0.0;
    axes_repub[4] = 0.0;
  }

  if (teleop_mode == "manipulator" && buttons[7]) {
    // vision arm layer (+)
    axes_repub.push_back(axes_repub[7]);
    axes_repub[7] = 0.0;
  } else {
    // normal arm layer
    axes_repub.push_back(0.0);
  }

  if (teleop_mode == "manipulator" && buttons[6]) {
    // vision arm layer (ABXY)
    buttons_repub.push_back(buttons_repub[0]);
    buttons_repub.push_back(buttons_repub[1]);
    buttons_repub.push_back(buttons_repub[2]);
    buttons_repub.push_back(buttons_repub[3]);

    buttons_repub[0] = 0;
    buttons_repub[1] = 0;
    buttons_repub[2] = 0;
    buttons_repub[3] = 0;
  } else {
    buttons_repub.push_back(0);
    buttons_repub.push_back(0);
    buttons_repub.push_back(0);
    buttons_repub.push_back(0);
  }

  joy_pub->publish(joy_repub);
  teleop_mode_pub->publish(mode_msg);
}