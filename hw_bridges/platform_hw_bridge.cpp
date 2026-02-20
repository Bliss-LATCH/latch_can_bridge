#include <cstring>
#include <latch_interfaces/msg/detail/can_frame__struct.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <memory.h>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

#include "latch_interfaces/msg/can_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::placeholders;

class PlatformHardwareBridge : public rclcpp::Node {
 public:
  PlatformHardwareBridge() : Node("front_tred_hw_bridge") {
    hrz_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "platform_front/horizontal_angle", 10,
        std::bind(&PlatformHardwareBridge::hrz_angle_callback, this, _1));

    can_rx_sub_ = this->create_subscription<latch_interfaces::msg::CanFrame>(
        "can/rx", 10,
        std::bind(&PlatformHardwareBridge::can_rx_callback, this, _1));

    can_tx_pub_ =
        this->create_publisher<latch_interfaces::msg::CanFrame>("can/tx", 10);
  }

 private:
  // Horizontal and vertical platform angle sub
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hrz_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vrt_angle_sub_;

  // CAN rx subscriber
  rclcpp::Subscription<latch_interfaces::msg::CanFrame>::SharedPtr can_rx_sub_;

  // CAN tx publisher
  rclcpp::Publisher<latch_interfaces::msg::CanFrame>::SharedPtr can_tx_pub_;

  void hrz_angle_callback(const std_msgs::msg::Float32::SharedPtr angle){
    latch_interfaces::msg::CanFrame frame;
    frame.can_id = 0x226; // TODO pull from a config YAML
    frame.data_len = 4;
    memcpy(&frame.data, &angle->data, sizeof(angle->data));

    can_tx_pub_->publish(frame);
  };

  // CAN rx callback
  void can_rx_callback(const latch_interfaces::msg::CanFrame::SharedPtr frame){
      // Nothing to implement currently
  };
};