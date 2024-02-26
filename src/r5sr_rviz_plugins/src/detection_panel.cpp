#include "detection_panel.hpp"

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

namespace r5sr_rviz_plugins
{
DetectionPanel::DetectionPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  QFormLayout * layout = new QFormLayout;

  barcode_label = new QLabel(this);
  layout->addRow("barcode", barcode_label);

  this->setLayout(layout);
}

void DetectionPanel::onInitialize()
{
  node = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  barcode_sub = node->create_subscription<std_msgs::msg::String>(
    "barcode", 1, std::bind(&DetectionPanel::handle_barcode, this, std::placeholders::_1));
}

void DetectionPanel::handle_barcode(const std_msgs::msg::String::SharedPtr msg)
{
  barcode_label->setText(QString::fromStdString(msg->data));
}

void DetectionPanel::save(rviz_common::Config config) const { rviz_common::Panel::save(config); }

void DetectionPanel::load(const rviz_common::Config & config) { rviz_common::Panel::load(config); }

}  // namespace r5sr_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(r5sr_rviz_plugins::DetectionPanel, rviz_common::Panel)