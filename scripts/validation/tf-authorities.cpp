// Capture actual DDS publisher identities; Humble rclpy omits publisher GIDs.
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <thread>

std::string hex(const uint8_t *data, size_t size) {
  std::ostringstream out;
  for (size_t i = 0; i < size; ++i) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(data[i]);
  return out.str();
}
int main(int argc, char **argv) {
  const std::string ns = argc > 1 ? argv[1] : "/red_standard_robot1";
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("stage2_tf_authority_capture");
  std::set<std::string> rows;
  auto callback = [&](tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo &info) {
    const auto &gid = info.get_rmw_message_info().publisher_gid;
    for (const auto &tf : msg->transforms) {
      rows.insert("TF\t" + tf.header.frame_id + "\t" + tf.child_frame_id + "\t" + hex(gid.data, RMW_GID_STORAGE_SIZE));
    }
  };
  auto dynamic = node->create_subscription<tf2_msgs::msg::TFMessage>(
      ns + "/tf", rclcpp::QoS(100), callback);
  auto statics = node->create_subscription<tf2_msgs::msg::TFMessage>(
      ns + "/tf_static", rclcpp::QoS(1).transient_local(), callback);
  auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  for (const auto &topic : {ns + "/tf", ns + "/tf_static"}) {
    for (const auto &pub : node->get_publishers_info_by_topic(topic)) {
      auto gid = pub.endpoint_gid();
      std::cout << "PUB\t" << hex(gid.data(), gid.size()) << "\t" << pub.node_namespace() << "\t" << pub.node_name() << "\n";
    }
  }
  for (const auto &row : rows) std::cout << row << "\n";
  rclcpp::shutdown();
}
