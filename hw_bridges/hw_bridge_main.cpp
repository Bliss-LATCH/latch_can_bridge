#include "elevation_hw_bridge.cpp"
#include "platform_hw_bridge.cpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlatformHardwareBridge>());
  rclcpp::spin(std::make_shared<ElevationHardwareBridge>());
  rclcpp::shutdown();
  return 0;
}