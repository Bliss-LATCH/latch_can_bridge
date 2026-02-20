#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

#include "latch_interfaces/msg/can_frame.hpp"
#include "latch_interfaces/msg/front_tred.hpp"
#include "latch_interfaces/msg/wheel_data.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;

class ExampleHWBridge : public rclcpp::Node {
public:
    ExampleHWBridge() : Node("front_tred_hw_bridge") {
        front_tred_pub_ = this->create_publisher<latch_interfaces::msg::FrontTred>("telemetry/front_tred", 10);

        can_rx_sub_ = this->create_subscription<latch_interfaces::msg::CanFrame>(
            "can/rx", 10, std::bind(&ExampleHWBridge::can_rx_callback, this, _1));

        can_tx_pub_ = this->create_publisher<latch_interfaces::msg::CanFrame>("can/tx", 10);

        test_can_tx_loop();
    }

private:
    // wheel data publisher
    rclcpp::Publisher<latch_interfaces::msg::FrontTred>::SharedPtr front_tred_pub_;

    // CAN rx subscriber
    rclcpp::Subscription<latch_interfaces::msg::CanFrame>::SharedPtr can_rx_sub_;

    // CAN tx publisher
    rclcpp::Publisher<latch_interfaces::msg::CanFrame>::SharedPtr can_tx_pub_;

    // CAN rx callback
    void can_rx_callback(const latch_interfaces::msg::CanFrame::SharedPtr frame) {
        // id filtering
        if (frame->can_id != 0x120) return;

        latch_interfaces::msg::FrontTred msg;
        latch_interfaces::msg::WheelData left;
        latch_interfaces::msg::WheelData right;

        // temp 5 bytes of data
        left.encoder_ticks = frame->data[0];
        left.rmp = frame->data[1];
        right.encoder_ticks = frame->data[2];
        right.rmp = frame->data[3];
        msg.left = left;
        msg.right = right;
        msg.heading = frame->data[4];

        front_tred_pub_->publish(msg);
    };

    void test_can_tx_loop() {
        while (rclcpp::ok()) {
            latch_interfaces::msg::CanFrame frame;
            frame.can_id = 0x120;
            frame.data_len = 4;
            frame.data = {0x12, 0x15, 0x16, 0x33};

            can_tx_pub_->publish(frame);
        }
    }
};