#include "bridge.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  for (auto direction : {ros_gz_bridge::BridgeDirection::GZ_TO_ROS,
      ros_gz_bridge::BridgeDirection::ROS_TO_GZ, ros_gz_bridge::BridgeDirection::BIDIRECTIONAL})
  {
    auto bridge = std::make_shared<intramural_sim::Bridge>();
    ros_gz_bridge::BridgeConfig config;
    config.ros_topic_name = "/bridge_lifetime";
    config.gz_topic_name = "/bridge_lifetime";
    config.ros_type_name = "std_msgs/msg/String";
    config.gz_type_name = "ignition.msgs.StringMsg";
    config.direction = direction;
    bridge->add_bridge(config);
    std::weak_ptr<intramural_sim::Bridge> lifetime = bridge;
    intramural_sim::close_bridge(bridge);
    if (!lifetime.expired()) {
      std::cerr << "Bridge retained after close" << std::endl;
      return 1;
    }
  }
  rclcpp::shutdown();
  return 0;
}
