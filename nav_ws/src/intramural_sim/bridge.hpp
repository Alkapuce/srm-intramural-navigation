#pragma once
#include <chrono>
#include <thread>
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

inline void close_bridge(std::shared_ptr<Bridge> & bridge)
{
  auto node = bridge->get_node_base_interface()->get_shared_rcl_node_handle();
  bridge->close();
  bridge.reset();
  // Transport callbacks must release ROS entities before DDS global destruction starts.
  while (node.use_count() > 1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
}  // namespace intramural_sim
