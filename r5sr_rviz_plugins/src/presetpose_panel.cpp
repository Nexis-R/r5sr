#include "presetpose_panel.hpp"

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

namespace r5sr_rviz_plugins {
PresetposePanel::PresetposePanel(QWidget* parent) : rviz_common::Panel(parent) {
  auto* layout = new QVBoxLayout;

  auto initial_pose_button = new QPushButton("initial", this);
  auto high_pose_button = new QPushButton("high", this);

  layout->addWidget(initial_pose_button);
  layout->addWidget(high_pose_button);
  this->setLayout(layout);

  connect(initial_pose_button, &QPushButton::clicked, this, [&]() {
    auto const target_pose = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.000;
      msg.position.x = 0.174;
      msg.position.y = 0.000;
      msg.position.z = 0.223;
      return msg;
    }();

    this->move_to(target_pose);
  });

  connect(high_pose_button, &QPushButton::clicked, this, [&]() {
    auto const target_pose = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.000;
      msg.position.x = 0.174;
      msg.position.y = 0.000;
      msg.position.z = 0.423;
      return msg;
    }();

    this->move_to(target_pose);
  });
}

void PresetposePanel::onInitialize() {
  node =
      this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  move_group_interface =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          moveit::planning_interface::MoveGroupInterface(node, "r5sr_arm"));
}

void PresetposePanel::move_to(const geometry_msgs::msg::Pose& pose) {
  move_group_interface->setPoseTarget(pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface->plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    move_group_interface->execute(plan);
  } else {
  }
}

void PresetposePanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
}

void PresetposePanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
}
}  // namespace r5sr_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(r5sr_rviz_plugins::PresetposePanel, rviz_common::Panel)
