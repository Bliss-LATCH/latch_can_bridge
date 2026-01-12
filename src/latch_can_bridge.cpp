#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/parser.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "latch_can_bridge/latch_can_bridge.hpp"

LatchCanBridgeNode::LatchCanBridgeNode(const rclcpp::NodeOptions& options) : Node("latch_can_bridge", options) {
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
        // TODO handle errors
    }

    // create the can config yaml node
    auto pkg_path = ament_index_cpp::get_package_share_directory("latch_can_bridge");
    auto config_path = pkg_path + "/config/can_mappings.yaml";
    can_config_ = YAML::LoadFile(config_path);

    // Give the request the name of the desired can interface
    strcpy(interface_req_.ifr_name, can_config_["can_bridge"]["can_interface"].as<std::string>().c_str());

    load_can_mappings();
}

void LatchCanBridgeNode::load_can_mappings() {
    // select the mappings element from the can_bridge section of yaml
    auto can_mappings = can_config_["can_bridge"]["id_mappings"];

    // loop through the mappings
    for (auto mapped_id : can_mappings) {
        // get the id
        uint16_t can_id = mapped_id["can_id"].as<uint16_t>();
        // get the topic name
        std::string topic = mapped_id["topic"].as<std::string>();
        // get the type
        std::string msg_type = mapped_id["type"].as<std::string>();

        // create a publisher and map it to id
        if (msg_type == "int32") {
            publishers_int32_[can_id] = this->create_publisher<std_msgs::msg::Int32>(topic, 10);
        } else {
            // TODO implement error handleing and other publisher types
        }
    }
}

void LatchCanBridgeNode::can_listener_loop() {
    struct can_frame frame;
    auto can_bytes = read(socket_, &frame, sizeof(struct can_frame));
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LatchCanBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
