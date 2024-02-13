#ifndef DETECTION_PANEL_H
#define DETECTION_PANEL_H

#ifndef Q_MOC_RUN
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "std_msgs/msg/string.hpp"
#endif

namespace r5sr_rviz_plugins {

class DetectionPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  DetectionPanel(QWidget *parent = nullptr);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

 protected:
  rclcpp::Node::SharedPtr node;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr barcode_sub;

  void handle_barcode(const std_msgs::msg::String::SharedPtr msg);

  QLabel *barcode_label;
};

}  // namespace r5sr_rviz_plugins

#endif