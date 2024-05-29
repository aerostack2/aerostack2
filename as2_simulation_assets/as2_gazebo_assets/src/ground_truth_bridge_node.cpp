#include "as2_gazebo_assets/ground_truth_bridge.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthBridge>());
  rclcpp::shutdown();
  return 0;
}
