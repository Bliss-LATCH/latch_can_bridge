#include "elevation_hw_bridge.cpp"
#include "platform_hw_bridge.cpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto platform_node = std::make_shared<PlatformHardwareBridge>();
  auto elevation_node = std::make_shared<ElevationHardwareBridge>();

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(platform_node);
  executor.add_node(elevation_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}