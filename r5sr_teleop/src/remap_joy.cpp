#include "r5sr_teleop/remap_joy.hpp"

using namespace r5sr_teleop;

RemapJoy::RemapJoy()
    : Node("remap_joy"), teleop_mode("crawler"), is_emergency_stopped(false) {
  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1, std::bind(&RemapJoy::handle_joy, this, std::placeholders::_1));
  is_emergency_stopped_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/is_emergency_stopped", 1,
      std::bind(&RemapJoy::handle_is_emergency_stopped, this,
                std::placeholders::_1));

  teleop_mode_pub =
      this->create_publisher<std_msgs::msg::String>("teleop_mode", 1);
  joy_pub = this->create_publisher<sensor_msgs::msg::Joy>("out", 1);
}

void RemapJoy::handle_is_emergency_stopped(
    const std_msgs::msg::Bool::SharedPtr is_stopped) {
  is_emergency_stopped = is_stopped->data;
}

void RemapJoy::handle_joy(const sensor_msgs::msg::Joy::SharedPtr joy) {
  const auto& buttons = joy->buttons;

  if (buttons[11]) {
    teleop_mode = "crawler";
  } else if (buttons[12]) {
    teleop_mode = "manipulator";
  }

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

  if (teleop_mode == "manipulator" && buttons[9]) {
    // vision arm layer (+)
    axes_repub.push_back(axes_repub[7]);
    axes_repub[7] = 0.0;
  } else {
    // normal arm layer
    axes_repub.push_back(0.0);
  }

  if (teleop_mode == "manipulator" && buttons[8]) {
    // vision arm layer (ABXY)
    buttons_repub.push_back(buttons_repub[0]);
    buttons_repub.push_back(buttons_repub[1]);
    buttons_repub.push_back(buttons_repub[2]);
    buttons_repub.push_back(buttons_repub[3]);
    buttons_repub.push_back(buttons_repub[4]);
    buttons_repub.push_back(buttons_repub[5]);

    buttons_repub[0] = 0;
    buttons_repub[1] = 0;
    buttons_repub[2] = 0;
    buttons_repub[3] = 0;
    buttons_repub[4] = 0;
    buttons_repub[5] = 0;
  } else {
    buttons_repub.push_back(0);
    buttons_repub.push_back(0);
    buttons_repub.push_back(0);
    buttons_repub.push_back(0);
    buttons_repub.push_back(0);
    buttons_repub.push_back(0);
  }

  if (teleop_mode == "crawler" && buttons[4]) {
    // L1
    axes_repub.push_back(axes_repub[4]);
    axes_repub[3] = 0.0;
    buttons_repub[4] = 0;
  } else {
    axes_repub.push_back(0.0);
  }
  if (teleop_mode == "crawler" && buttons[5]) {
    // R1
    axes_repub.push_back(axes_repub[4]);
    axes_repub[3] = 0.0;
    buttons_repub[5] = 0;
  } else {
    axes_repub.push_back(0.0);
  }
  if (teleop_mode == "crawler" && buttons[6]) {
    // L2
    axes_repub.push_back(axes_repub[4]);
    axes_repub[3] = 0.0;
    buttons_repub[6] = 0;
  } else {
    axes_repub.push_back(0.0);
  }
  if (teleop_mode == "crawler" && buttons[7]) {
    // R2
    axes_repub.push_back(axes_repub[4]);
    axes_repub[3] = 0.0;
    buttons_repub[7] = 0;
  } else {
    axes_repub.push_back(0.0);
  }

  if (is_emergency_stopped) {
    for (auto& axis : axes_repub) {
      axis = 0.0;
    }
    axes_repub[2] = 1.0;
    axes_repub[5] = 1.0;
    for (auto& button : buttons_repub) {
      button = 0;
    }
    teleop_mode = "EMERGENCY STOP";
  }

  joy_pub->publish(joy_repub);

  std_msgs::msg::String mode_msg;
  mode_msg.data = teleop_mode;
  teleop_mode_pub->publish(mode_msg);
}