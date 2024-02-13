#ifndef PRESET_POSE_H
#define PRESET_POSE_H

#ifndef Q_MOC_RUN

#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#endif

#include <std_srvs/srv/empty.hpp>

namespace r5sr_rviz_plugins {
class PresetposePanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  PresetposePanel(QWidget *parent = nullptr);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

 protected:
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr move_to_default_pose_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr move_to_floor_pose_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr move_to_high_pose_client;

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr move_to_dex1_center_pose_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr move_to_dex1_diaright_pose_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr move_to_dex1_dialeft_pose_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr move_to_dex1_right_pose_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr move_to_dex1_left_pose_client;
};
}  // namespace r5sr_rviz_plugins

#endif