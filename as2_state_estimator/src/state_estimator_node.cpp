#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include "as2_state_estimator/state_estimator.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
