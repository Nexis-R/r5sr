#include "mode_panel.hpp"

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

namespace r5sr_rviz_plugins {
ModePanel::ModePanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto *layout = new QHBoxLayout;

  auto emergencyButton = new QPushButton("EMERGENCY STOP", this);
  emergencyButton->setCheckable(true);
  auto palette = emergencyButton->palette();
  palette.setColor(QPalette::Button, QColor(Qt::red));
  emergencyButton->setPalette(palette);

  layout->addWidget(emergencyButton);
  this->setLayout(layout);

  timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this,
          [&]() { is_stopped_pub->publish(is_stopped); });
  connect(emergencyButton, &QPushButton::toggled, this,
          [&](bool is_checked) { is_stopped.set__data(is_checked); });
  connect(emergencyButton, &QPushButton::toggled, emergencyButton,
          &QPushButton::setDisabled);
}

void ModePanel::onInitialize() {
  is_stopped.set__data(false);

  node =
      this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  is_stopped_pub =
      node->create_publisher<std_msgs::msg::Bool>("is_emergency_stopped", 1);

  timer->start(500);
}

void ModePanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
}

void ModePanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
}

}  // namespace r5sr_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(r5sr_rviz_plugins::ModePanel, rviz_common::Panel)