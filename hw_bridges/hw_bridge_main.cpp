#include "platform_hw_bridge.cpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlatformHardwareBridge>());
    rclcpp::shutdown();
    return 0;
}