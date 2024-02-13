#ifndef R5SR_RVIZ_PLUGINS_H
#define R5SR_RVIZ_PLUGINS_H

#ifndef Q_MOC_RUN
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "std_msgs/msg/bool.hpp"
#endif

namespace r5sr_rviz_plugins {

class ModePanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  ModePanel(QWidget *parent = nullptr);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

 protected:
  QTimer *timer;
  std_msgs::msg::Bool is_stopped;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_stopped_pub;
};

}  // namespace r5sr_rviz_plugins

#endif