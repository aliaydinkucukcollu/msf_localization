#include "../include/msf_localization_ros2/msf_localization_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<msf_localization::MSFLocalization>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
