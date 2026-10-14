// Inspect native service reply QoS rather than assuming XML was applied.
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rcl/service.h>
#include <rmw_fastrtps_cpp/get_service.hpp>
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("service_qos_probe");
  auto service = node->create_service<std_srvs::srv::Trigger>("service_qos_probe",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {response->success = true;});
  auto native = rcl_service_get_rmw_handle(service->get_service_handle().get());
  auto writer = rmw_fastrtps_cpp::get_response_datawriter(const_cast<rmw_service_t *>(native));
  if (!writer) return 2;
  auto qos = writer->get_qos().reliability();
  int64_t nanoseconds = static_cast<int64_t>(qos.max_blocking_time.seconds) * 1000000000LL + qos.max_blocking_time.nanosec;
  std::cout << "{\"response_max_blocking_time_ns\":" << nanoseconds << "}" << std::endl;
  service.reset(); node.reset(); rclcpp::shutdown();
  return nanoseconds == 5000000000LL ? 0 : 1;
}
