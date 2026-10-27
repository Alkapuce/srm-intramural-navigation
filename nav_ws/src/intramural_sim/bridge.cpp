#include "bridge.hpp"
#include <exception>
#include <iostream>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto bridge = std::make_shared<intramural_sim::Bridge>();
  int result = 0;
  try {
    rclcpp::spin(bridge);
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    result = 1;
  }
  intramural_sim::close_bridge(bridge);
  // Join deferred signal handling before logger / DDS shared libraries are unloaded.
  rclcpp::shutdown();
  return result;
}
