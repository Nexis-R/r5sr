#ifndef PRESET_POSE_H
#define PRESET_POSE_H

#ifndef Q_MOC_RUN

#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "std_msgs/msg/bool.hpp"
#endif

#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>

namespace r5sr_rviz_plugins {
class PresetposePanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  PresetposePanel(QWidget *parent = nullptr);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

 protected:
  void move_to(const geometry_msgs::msg::Pose &pose);
  rclcpp::Node::SharedPtr node;
  moveit::planning_interface::MoveGroupInterfaceUniquePtr move_group_interface;
};
}  // namespace r5sr_rviz_plugins

#endif