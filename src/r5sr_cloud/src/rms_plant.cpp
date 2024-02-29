#include "r5sr_cloud/rms_plant.hpp"

#include <cpprest/details/basic_types.h>
#include <cpprest/http_msg.h>
#include <cpprest/json.h>
#include <cpprest/uri_builder.h>

#include <chrono>
#include <rclcpp/logging.hpp>

namespace r5sr_cloud
{
using namespace web;
using namespace web::http;

RmsPlant::RmsPlant(const rclcpp::NodeOptions & options) : Node("rms_plant", options), last_req_id(0)
{
  declare_parameter("server_ip", "0.0.0.0");
  declare_parameter("plant_name", "");
  declare_parameter("site.id", 0);
  declare_parameter("site.name", "");
  declare_parameter("user_id", 0);
  declare_parameter("polling_interval_ms", 1000);

  site_id = get_parameter("site.id").as_int();
  site_name = get_parameter("site.name").as_string();
  plant_name = get_parameter("plant_name").as_string();
  RCLCPP_INFO_STREAM(get_logger(), "Site ID:" << site_id);
  RCLCPP_INFO_STREAM(get_logger(), "Site Name:" << site_name);
  RCLCPP_INFO_STREAM(get_logger(), "Plant Name:" << plant_name);
  RCLCPP_INFO_STREAM(get_logger(), "This plant site is under operation...");

  server_uri = "http://" + get_parameter("server_ip").as_string();
  client = std::make_unique<web::http::client::http_client>(U(server_uri));
  uri_builder builder{U("/rms_v2/api/get_site_init.php")};

  json::value initVal;
  initVal[U("site_id")] = site_id;
  try {
    client->request(methods::POST, builder.to_string(), initVal.serialize(), U("application/json"))
      .then([this](http_response response) {
        if (response.status_code() == status_codes::OK) {
          RCLCPP_INFO_STREAM(get_logger(), response.to_string());
          return response.extract_string();
        } else {
          return pplx::task_from_result(std::string());
        }
      })
      .then([this](const std::string & body) {
        const auto ret = json::value::parse(body);
        const bool is_ok = ret.at(U("status")).as_string() == "ok";
        if (is_ok) {
          floor_length = ret.at(U("length_x")).as_string();
          floor_width = ret.at(U("length_y")).as_string();
          origin_x = ret.at(U("origin_x")).as_string();
          origin_y = ret.at(U("origin_y")).as_string();

          uri_floor_map = server_uri + "/rms_v2/uploads/" + ret.at(U("fmap")).as_string();
          uri_pnid = server_uri + "/rms_v2/uploads/" + ret.at(U("pnid")).as_string();
          RCLCPP_INFO_STREAM(get_logger(), "connected to server...");
          RCLCPP_INFO_STREAM(get_logger(), "Floor map: " << uri_floor_map);
          RCLCPP_INFO_STREAM(get_logger(), "PNID: " << uri_pnid);
          RCLCPP_INFO_STREAM(get_logger(), "Length: " << floor_length);
          RCLCPP_INFO_STREAM(get_logger(), "Width: " << floor_width);
          RCLCPP_INFO_STREAM(get_logger(), "Origin X: " << origin_x);
          RCLCPP_INFO_STREAM(get_logger(), "Origin Y: " << origin_y);
          polling_timer = create_wall_timer(
            std::chrono::milliseconds(get_parameter("polling_interval_ms").as_int()),
            std::bind(&RmsPlant::poll, this));
        }
      })
      .wait();
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error: " << e.what());
  }
}

void RmsPlant::poll()
{
  json::value site_req_value;
  site_req_value[U("ent_type")] = json::value::string(U("pt"));
  site_req_value[U("ent_id")] = json::value::number(site_id);
  site_req_value[U("last_req_id")] = json::value::number(last_req_id);

  try {
    client
      ->request(
        methods::POST, U("/rms_v2/api/get_ent_req.php"), site_req_value.serialize(),
        U("application/json"))
      .then([](http_response response) {
        if (response.status_code() == status_codes::OK) {
          return response.extract_string();
        } else {
          return pplx::task_from_result(std::string());
        }
      })
      .then([this](const std::string & body) {
        const auto ret = json::value::parse(body);
        const bool is_ok = ret.at(U("status")).as_string() == "ok";
        if (is_ok) {
          last_req_id = std::stoi(ret.at(U("req_id")).as_string());

          const auto req_jsn = json::value::parse(ret.at(U("req_jsn")).as_string());
          if (req_jsn.at(U("req_id")).as_string() == "send_site_info") {
            // Do something with results...
            json::value site_info;
            site_info[U("site_id")] = json::value::number(site_id);
            site_info[U("site_name")] = json::value::string(site_name);
            site_info[U("plant_name")] = json::value::string(plant_name);
            site_info[U("lngth")] = json::value::string(floor_length);
            site_info[U("width")] = json::value::string(floor_width);
            site_info[U("x0")] = json::value::string(origin_x);
            site_info[U("y0")] = json::value::string(origin_y);
            site_info[U("map2d")] = json::value::string(uri_floor_map);
            site_info[U("pnid")] = json::value::string(uri_pnid);

            json::value r_values;
            r_values[U("req_id")] = ret.at(U("req_id"));
            r_values[U("erq_id")] = ret.at(U("erq_id"));
            r_values[U("from_ent_type")] = ret.at(U("from_ent_type"));
            r_values[U("from_ent_id")] = ret.at(U("from_ent_id"));
            r_values[U("res_jsn")] = json::value::string(site_info.serialize());

            client
              ->request(
                methods::POST, U("/rms_v2/api/res_site_info.php"), r_values.serialize(),
                U("application/json"))
              .then([](http_response response) {
                if (response.status_code() == status_codes::OK) {
                  return response.extract_string();
                } else {
                  return pplx::task_from_result(std::string());
                }
              })
              .then([this](const std::string & body) {
                RCLCPP_INFO_STREAM(get_logger(), "Response: " << body);
              })
              .wait();
          }
        }
      })
      .wait();
  } catch (const std::exception & e) {
  }
}
}  // namespace r5sr_cloud

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(r5sr_cloud::RmsPlant)