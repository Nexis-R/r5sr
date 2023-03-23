#include "presetpose_panel.hpp"

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

namespace r5sr_rviz_plugins {
PresetposePanel::PresetposePanel(QWidget* parent) : rviz_common::Panel(parent) {
  auto* layout = new QVBoxLayout;

  auto initial_pose_button = new QPushButton("initial", this);
  auto floor_pose_button = new QPushButton("floor", this);
  auto high_pose_button = new QPushButton("high", this);

  layout->addWidget(initial_pose_button);
  layout->addWidget(floor_pose_button);
  layout->addWidget(high_pose_button);

  this->setLayout(layout);

  connect(initial_pose_button, &QPushButton::clicked, this, [&]() {
    move_to_default_pose_client->async_send_request(
        std::make_shared<std_srvs::srv::Empty::Request>());
  });

  connect(floor_pose_button, &QPushButton::clicked, this, [&]() {
    move_to_floor_pose_client->async_send_request(
        std::make_shared<std_srvs::srv::Empty::Request>());
  });

  connect(high_pose_button, &QPushButton::clicked, this, [&]() {
    move_to_high_pose_client->async_send_request(
        std::make_shared<std_srvs::srv::Empty::Request>());
  });
}

void PresetposePanel::onInitialize() {
  node =
      this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  move_to_default_pose_client = node->create_client<std_srvs::srv::Empty>(
      "preset_pose_node/move_to_default_pose");
  move_to_floor_pose_client = node->create_client<std_srvs::srv::Empty>(
      "preset_pose_node/move_to_floor_pose");
  move_to_high_pose_client = node->create_client<std_srvs::srv::Empty>(
      "preset_pose_node/move_to_high_pose");
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
