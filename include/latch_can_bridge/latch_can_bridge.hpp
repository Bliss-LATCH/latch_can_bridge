#pragma once

#include <memory>
#include <rclcpp/node_options.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace latch_can_bridge {

class LatchCanBridgeNode : public rclcpp::Node {
public:
    explicit LatchCanBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    

private:
};

}  // namespace latch_can_bridge
