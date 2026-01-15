#pragma once

#include <linux/can.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>

#include <cstdint>
#include <memory>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/node_options.hpp>
#include <string>
#include <thread>
#include <unordered_map>

#include "latch_interfaces/msg/can_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

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

    // can rx publisher
    rclcpp::Publisher<latch_interfaces::msg::CanFrame>::SharedPtr can_rx_pub_;

    // function defs
    void can_listener_loop();
};