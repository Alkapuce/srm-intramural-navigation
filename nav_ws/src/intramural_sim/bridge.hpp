#pragma once
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

namespace intramural_sim
{
class Bridge : public ros_gz_bridge::RosGzBridge
{
public:
  void close()
  {
    // Humble 0.244.25 retains node -> handles -> node cycles past middleware teardown.
    heartbeat_timer_->cancel();
    handles_.clear();
    services_.clear();
    gz_node_.reset();
  }
};
}  // namespace intramural_sim
