#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace r5sr_teleop
{
class FlirAx8Rtsp : public rclcpp::Node
{
public:
  FlirAx8Rtsp();

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compresed_image_pub;
  rclcpp::TimerBase::SharedPtr timer;
  cv::VideoCapture cap;
  sensor_msgs::msg::Image image_msg;
  void handle_image();
};
}  // namespace r5sr_teleop