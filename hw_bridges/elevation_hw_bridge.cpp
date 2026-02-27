#include <memory.h>

#include <cstring>
#include <latch_interfaces/msg/detail/can_frame__struct.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <unordered_map>
#include <vector>

#include "latch_can_bridge/can_yaml_utils.hpp"
#include "latch_interfaces/msg/can_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class ElevationHardwareBridge : public rclcpp::Node {
 public:
  ElevationHardwareBridge() : Node("elevation_hw_bridge") {
    auto jetson_map = getDeviceIDMap("jetson");
    auto elevation_front_map = getDeviceIDMap("elevation_front");
    auto elevation_back_map = getDeviceIDMap("elevation_back");

    // Create subscriptions for all angle topics
    for (const auto& [msg_name, id_mapping] : jetson_map) {
      if (msg_name.find("elevation") == std::string::npos) {
        continue;  // Skip non-elevation messages
      }
      auto sub = this->create_subscription<std_msgs::msg::Float32>(
          id_mapping.topic, 10,
          [this, can_id = id_mapping.can_id](
              const std_msgs::msg::Float32::SharedPtr msg) {
            this->height_callback(msg, can_id);
          });
      subscriptions_.push_back(sub);
    }

    // Create publishers for all CAN IDs in elevation maps
    for (const auto& [msg_name, id_mapping] : elevation_front_map) {
      auto pub =
          this->create_publisher<std_msgs::msg::Float32>(id_mapping.topic, 10);
      canid_publishers_[id_mapping.can_id] = pub;
    }

    for (const auto& [msg_name, id_mapping] : elevation_back_map) {
      auto pub =
          this->create_publisher<std_msgs::msg::Float32>(id_mapping.topic, 10);
      canid_publishers_[id_mapping.can_id] = pub;
    }

    // Setup CAN communication
    can_rx_sub_ = this->create_subscription<latch_interfaces::msg::CanFrame>(
        "can/rx", 10,
        [this](const latch_interfaces::msg::CanFrame::SharedPtr msg) {
          this->can_rx_callback(msg);
        });

    can_tx_pub_ =
        this->create_publisher<latch_interfaces::msg::CanFrame>("can/tx", 10);
  };

 private:
  // Dynamic storage for subscriptions
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr>
      subscriptions_;

  // Map of CAN ID to topic publishers for received data
  std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr>
      canid_publishers_;

  // CAN rx subscriber
  rclcpp::Subscription<latch_interfaces::msg::CanFrame>::SharedPtr can_rx_sub_;

  // CAN tx publisher
  rclcpp::Publisher<latch_interfaces::msg::CanFrame>::SharedPtr can_tx_pub_;

  // Unified height callback that publishes the desired height of the system to
  // CAN
  void height_callback(const std_msgs::msg::Float32::SharedPtr height,
                       int can_id) {
    latch_interfaces::msg::CanFrame frame;
    frame.can_id = can_id;
    frame.data_len = 4;
    memcpy(&frame.data, &height->data, sizeof(height->data));
    can_tx_pub_->publish(frame);
  };

  // CAN rx callback
  void can_rx_callback(const latch_interfaces::msg::CanFrame::SharedPtr frame) {
    auto it = canid_publishers_.find(frame->can_id);
    if (it != canid_publishers_.end()) {
      std_msgs::msg::Float32 msg;
      memcpy(&msg.data, &frame->data, sizeof(msg.data));
      it->second->publish(msg);
    }
  }
};