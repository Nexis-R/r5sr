#include "r5sr_teleop/flir_ax8_rtsp.hpp"

namespace r5sr_teleop {
FlirAx8Rtsp::FlirAx8Rtsp() : Node("flir_ax8_rtsp") {
  this->declare_parameter("ip_addr", "192.168.1.22");
  this->declare_parameter("encoding", "mpeg4");
  this->declare_parameter("text_overlay", "off");
  this->declare_parameter("rate_hz", 5);

  image_pub =
      this->create_publisher<sensor_msgs::msg::Image>("filr_ax8_raw", 1);
  cap.open("rtsp://" + this->get_parameter("ip_addr").as_string() + "/" +
           this->get_parameter("encoding").as_string() +
           "?overlay=" + this->get_parameter("text_overlay").as_string() +
           "?frate=" + std::to_string(this->get_parameter("rate_hz").as_int()));

  if (!cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
  } else {
    RCLCPP_INFO(this->get_logger(), "Opened camera");

    timer = this->create_wall_timer(
        std::chrono::milliseconds(1000 /
                                  this->get_parameter("rate_hz").as_int()),
        std::bind(&FlirAx8Rtsp::handle_image, this));
  }
}

void FlirAx8Rtsp::handle_image() {
  cap >> frame;
  if (frame.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to capture frame");
    return;
  }

  image_msg.header.stamp = this->now();
  image_msg.header.frame_id = "flir_ax8";
  image_msg.height = frame.rows;
  image_msg.width = frame.cols;
  image_msg.encoding = "bgr8";
  image_msg.is_bigendian = false;
  image_msg.step = frame.cols * frame.elemSize();
  image_msg.data.resize(image_msg.step * image_msg.height);
  memcpy(image_msg.data.data(), frame.data, image_msg.data.size());

  image_pub->publish(image_msg);
}
}  // namespace r5sr_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<r5sr_teleop::FlirAx8Rtsp>());
  rclcpp::shutdown();
  return 0;
}