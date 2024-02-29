#ifndef R5SR_CLOUD__RMS_PLANT_HPP_
#define R5SR_CLOUD__RMS_PLANT_HPP_

#include <cpprest/http_client.h>

#include <cstdint>
#include <memory>
#include <rclcpp/timer.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace r5sr_cloud
{
class RmsPlant : public rclcpp::Node
{
public:
  explicit RmsPlant(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void poll();
  rclcpp::TimerBase::SharedPtr polling_timer;

  std::unique_ptr<web::http::client::http_client> client;

  int64_t site_id;
  std::string site_name;
  std::string plant_name;
  std::string uri_floor_map;
  std::string uri_pnid;
  std::string server_uri;
  int64_t last_req_id;

  std::string floor_length;
  std::string floor_width;
  std::string origin_x;
  std::string origin_y;
};
}  // namespace r5sr_cloud

#endif