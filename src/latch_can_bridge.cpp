#include <linux/can.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "latch_can_bridge/latch_can_bridge.hpp"
#include "latch_interfaces/msg/can_frame.hpp"

using namespace std::chrono_literals;

LatchCanBridgeNode::LatchCanBridgeNode(const rclcpp::NodeOptions& options) : Node("latch_can_bridge", options) {
    // setup CAN rx publisher
    can_rx_pub_ = this->create_publisher<latch_interfaces::msg::CanFrame>("can/rx", 10);

    // set the socket FD
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        // TODO handle errors
    }

    memcpy(&interface_req_.ifr_name, "can0", 4);
    ioctl(socket_fd_, SIOCGIFINDEX, &interface_req_);  // system call to get the interface for can0

    // bind the socket address to CAN interface
    socket_addr_.can_family = AF_CAN;
    socket_addr_.can_ifindex = interface_req_.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&socket_addr_, sizeof(socket_addr_)) < 0) {
        // TODO handle errors
    }

    listener_thread_ = std::thread(&LatchCanBridgeNode::can_listener_loop, this);
}

void LatchCanBridgeNode::can_listener_loop() {
    struct can_frame frame;
    auto nbytes = read(socket_fd_, &frame, sizeof(frame));

    if (nbytes < 0) {
        // TODO handle errors
    }

    latch_interfaces::msg::CanFrame msg;
    msg.can_id = frame.can_id;
    msg.data_len = frame.len;
    memcpy(msg.data.data(), frame.data, frame.len);

    can_rx_pub_->publish(msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LatchCanBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
