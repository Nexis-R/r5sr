#include <Definitions.h>

#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

class CrawlerNode : public rclcpp::Node {
 public:
  CrawlerNode() : Node("crawler_node") {
    this->declare_parameter<std::string>("device_name", "EPOS4");
    this->declare_parameter<std::string>("device_name_sub", "EPOS4");
    this->declare_parameter<std::string>("protocol_stack_name",
                                         "MAXON SERIAL V2");
    this->declare_parameter<std::string>("protocol_stack_name_sub", "CANopen");
    this->declare_parameter<std::string>("interface_name", "USB");
    this->declare_parameter<std::string>("port_name", "USB0");
    this->declare_parameter<uint8_t>("id_host", 1);
    this->declare_parameters<uint8_t>(
        "crawler_left", std::map<std::string, uint8_t>{{"id", 1}});
    this->declare_parameters<uint8_t>(
        "crawler_right", std::map<std::string, uint8_t>{{"id", 2}});

    openDevice();
    enableDevice();

    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&CrawlerNode::cmdVelCallback, this, std::placeholders::_1));
  }

  ~CrawlerNode() { closeDevice(); }

 private:
  void *keyHandleHost;
  void *keyHandleSub;

  void openDevice() {
    unsigned int err = 0;
    char *device_name =
        (char *)this->get_parameter("device_name").as_string().c_str();
    char *protocol_stack_name =
        (char *)this->get_parameter("protocol_stack_name").as_string().c_str();
    char *interface_name =
        (char *)this->get_parameter("interface_name").as_string().c_str();
    char *port_name =
        (char *)this->get_parameter("port_name").as_string().c_str();

    RCLCPP_INFO(this->get_logger(), "Opening host device...");
    keyHandleHost = VCS_OpenDevice(device_name, protocol_stack_name,
                                   interface_name, port_name, &err);

    if (keyHandleHost != 0 && err == 0) {
      unsigned int baudrate = 0;
      unsigned int timeout = 0;
      VCS_GetProtocolStackSettings(keyHandleHost, &baudrate, &timeout, &err);
      RCLCPP_INFO(this->get_logger(), "Host device opened successfully.");
      RCLCPP_INFO(this->get_logger(), "baudrate: %d", baudrate);
      RCLCPP_INFO(this->get_logger(), "timeout: %d", timeout);

      char *device_name_sub =
          (char *)this->get_parameter("device_name_sub").as_string().c_str();
      char *protocol_stack_name_sub =
          (char *)this->get_parameter("protocol_stack_name_sub")
              .as_string()
              .c_str();
      keyHandleSub = VCS_OpenSubDevice(keyHandleHost, device_name_sub,
                                       protocol_stack_name_sub, &err);

      if (keyHandleSub != 0 && err == 0) {
        unsigned int baudrate = 0;
        VCS_GetGatewaySettings(keyHandleHost, &baudrate, &err);
        RCLCPP_INFO(this->get_logger(), "Sub device opened successfully.");
        RCLCPP_INFO(this->get_logger(), "baudrate: %d", baudrate);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Error opening sub device.");
      }

    } else {
      RCLCPP_ERROR(this->get_logger(), "Error opening host device");
      RCLCPP_ERROR(
          this->get_logger(), "protocol stack name: %s",
          this->get_parameter("protocol_stack_name").as_string().c_str());
      RCLCPP_ERROR(this->get_logger(), "interface name: %s",
                   this->get_parameter("interface_name").as_string().c_str());
      RCLCPP_ERROR(this->get_logger(), "port name: %s",
                   this->get_parameter("port_name").as_string().c_str());
    }
  }

  void enableDevice() {
    const uint8_t id_host = this->get_parameter("id_host").as_int();
    std::vector<std::pair<void *, uint8_t>> nodes;
    nodes.push_back({keyHandleHost, id_host});
    std::array<uint8_t, 2> ids{
        {static_cast<uint8_t>(this->get_parameter("crawler_left.id").as_int()),
         static_cast<uint8_t>(
             this->get_parameter("crawler_right.id").as_int())}};
    for (const auto &id : ids) {
      if (id != id_host) nodes.push_back({keyHandleSub, id});
    }

    for (const auto &node : nodes) {
      void *keyHandle = node.first;
      const uint8_t id = node.second;

      unsigned int err = 0;
      int isFault = 0;

      if (VCS_GetFaultState(keyHandle, id, &isFault, &err) == 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting fault state.");
        RCLCPP_ERROR(this->get_logger(), "id: %d", id);
      }

      if (isFault && err == 0) {
        RCLCPP_INFO(this->get_logger(), "clearing fault state...");
        RCLCPP_INFO(this->get_logger(), "id: %d", id);
        if (VCS_ClearFault(keyHandle, id, &err) == 0) {
          RCLCPP_ERROR(this->get_logger(), "Error clearing fault.");
          RCLCPP_ERROR(this->get_logger(), "id: %d", id);
        }
      }

      int isEnabled = 0;

      if (VCS_GetEnableState(keyHandle, id, &isEnabled, &err) == 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting enable state.");
        RCLCPP_ERROR(this->get_logger(), "id: %d", id);
      }

      if (!isEnabled && err == 0) {
        RCLCPP_INFO(this->get_logger(), "enabling device...");
        RCLCPP_INFO(this->get_logger(), "id: %d", id);
        if (VCS_SetEnableState(keyHandle, id, &err) == 0) {
          RCLCPP_ERROR(this->get_logger(), "Error enabling device.");
          RCLCPP_ERROR(this->get_logger(), "id: %d", id);
        }
      }

      RCLCPP_INFO(this->get_logger(), "activating device...");
      RCLCPP_INFO(this->get_logger(), "id: %d", id);
      if (VCS_ActivateProfileVelocityMode(keyHandle, id, &err) == 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Error activating profile velocity mode.");
        RCLCPP_ERROR(this->get_logger(), "id: %d", id);
      }
    }
  }

  void closeDevice() {
    unsigned int err = 0;

    if (VCS_CloseSubDevice(keyHandleSub, &err) == 0) {
      RCLCPP_ERROR(this->get_logger(), "Error closing sub device.");
    }

    if (VCS_CloseDevice(keyHandleHost, &err) == 0) {
      RCLCPP_ERROR(this->get_logger(), "Error closing host device.");
    }
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    unsigned int err = 0;

    const double leftSpeed = msg->linear.x - msg->angular.z * 0.5;   // m/s
    const double rightSpeed = msg->linear.x + msg->angular.z * 0.5;  // m/s

    const double r = 0.1;                                      // m
    const double leftRpm = leftSpeed * 60 / (2 * M_PI * r);    // rpm
    const double rightRpm = rightSpeed * 60 / (2 * M_PI * r);  // rpm

    const uint8_t id_host = this->get_parameter("id_host").as_int();
    void *keyHandle;

    const uint8_t id_left = this->get_parameter("crawler_left.id").as_int();
    keyHandle = (id_host == id_left) ? keyHandleHost : keyHandleSub;
    if (VCS_MoveWithVelocity(keyHandle, id_left, leftRpm, &err) == 0) {
      RCLCPP_ERROR(this->get_logger(), "Error moving left crawler.");
      RCLCPP_ERROR(this->get_logger(), "id: %d", id_left);
    }

    const uint8_t id_right = this->get_parameter("crawler_right.id").as_int();
    keyHandle = (id_host == id_right) ? keyHandleHost : keyHandleSub;
    if (VCS_MoveWithVelocity(keyHandle, id_right, rightRpm, &err) == 0) {
      RCLCPP_ERROR(this->get_logger(), "Error moving right crawler.");
      RCLCPP_ERROR(this->get_logger(), "id: %d", id_right);
    }
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrawlerNode>());
  rclcpp::shutdown();
}