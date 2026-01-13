#pragma once

#include <linux/can.h>
#include <net/if.h>
#include <sys/socket.h>

#include <cstdint>
#include <memory>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/node_options.hpp>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "yaml-cpp/yaml.h"

class LatchCanBridgeNode : public rclcpp::Node {
public:
    explicit LatchCanBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // the can interface of the linux device "can0"
    std::string can_interface_;

    // the socket for the can interface
    int socket_fd_;
    struct sockaddr_can socket_addr_;

    // interface request var
    struct ifreq interface_req_;

    // can listener thread
    std::thread listener_thread_;

    // can config YAML file
    YAML::Node can_config_;

    // publisher types for the different can id associations
    std::unordered_map<uint16_t, std::shared_ptr<rclcpp::GenericPublisher>> publishers_;

    rclcpp::TimerBase::SharedPtr can_timer_;

    // function defs
    void load_can_mappings();
    void can_listener_callback();
};