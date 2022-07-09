#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <Definitions.h>

using namespace std::chrono_literals;

class CrawlerNode : public rclcpp::Node
{
public:
  CrawlerNode()
      : Node("crawler_node")
  {
    this->declare_parameter<std::string>("device_name", "EPOS4");
    this->declare_parameter<std::string>("protocol_stack_name", "MAXON SERIAL V2");
    this->declare_parameter<std::string>("interface_name", "USB");
    this->declare_parameter<std::string>("port_name", "USB0");
    this->declare_parameter<uint8_t>("id_host", 1);
    this->declare_parameters<uint8_t>("crawler_left", std::map<std::string, uint8_t>{{"id", 1}});
    this->declare_parameters<uint8_t>("crawler_right", std::map<std::string, uint8_t>{{"id", 2}});

    unsigned int err = 0;
    char* deviceName = (char*)this->get_parameter("device_name").as_string().c_str();
    VCS_OpenDevice(deviceName, "MAXON SERIAL V2", "USB", "USB0", &err);
  }

private:
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrawlerNode>());
  rclcpp::shutdown();
}