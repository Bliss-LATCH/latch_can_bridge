#include <memory>
#include <rclcpp/node_options.hpp>

#include "latch_can_bridge/latch_can_bridge.hpp"

namespace latch_can_bridge {
LatchCanBridgeNode::LatchCanBridgeNode(const rclcpp::NodeOptions& options) : Node("latch_can_bridge", options) {}
}  // namespace latch_can_bridge

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<latch_can_bridge::LatchCanBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
