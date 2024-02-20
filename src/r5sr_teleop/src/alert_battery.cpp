#include "rclcpp/rclcpp.hpp"
#include "rviz_2d_overlay_msgs/msg/overlay_text.hpp"
#include "std_msgs/msg/float32.hpp"

class VoltageMonitor : public rclcpp::Node
{
public:
  VoltageMonitor() : Node("voltage_monitor")
  {
    voltage_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "/MoveWithJointState/voltage", 10,
      std::bind(&VoltageMonitor::voltage_callback, this, std::placeholders::_1));

    voltage_publisher_ =
      this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("alert_battery_overlay", 10);

    threshold_voltage_ = 22.0;  // Set the voltage threshold
  }

private:
  void voltage_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float voltage = msg->data;

    rviz_2d_overlay_msgs::msg::OverlayText overlay_msg;

    overlay_msg.width = 470;
    overlay_msg.height = 50;
    overlay_msg.text_size = 30;
    overlay_msg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::CENTER;

    overlay_msg.bg_color.r = 0.0;
    overlay_msg.bg_color.g = 0.0;
    overlay_msg.bg_color.b = 0.0;
    overlay_msg.bg_color.a = 0.5;

    if (voltage <= threshold_voltage_) {
      overlay_msg.text = "Warning Low Voltage";
    } else {
      overlay_msg.text = "Safe Normal Voltage";
    }

    voltage_publisher_->publish(overlay_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr voltage_subscription_;
  rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr voltage_publisher_;

  double threshold_voltage_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto voltage_monitor = std::make_shared<VoltageMonitor>();
  rclcpp::spin(voltage_monitor);
  rclcpp::shutdown();
  return 0;
}
