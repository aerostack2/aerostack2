#ifndef MODULES__GO_TO_MODULE
#define MODULES__GO_TO_MODULE

#include "as2_cpp_api/behavior_actions/go_to_behavior.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2
{
namespace as2_cpp_api
{

class GoToModule
{
public:
  GoToModule();
  void initialize(const rclcpp::Node::SharedPtr & drone_interface_node);
  bool operator()(
    const double x,
    const double y,
    const double z,
    const double speed,
    const int yaw_mode,
    const double yaw_angle,
    const bool use_yaw_angle,
    const std::string & frame_id,
    const bool wait_result = true);

  bool go_to_point(
    const std::vector<double> & point, const double speed,
    const std::string & frame_id = "earth", const bool wait_result = true);
  bool go_to_point_path_facing(
    const std::vector<double> & point, const double speed,
    const std::string & frame_id = "earth",
    const bool wait_result = true);

private:
  bool __go_to(
    const double x,
    const double y,
    const double z,
    const double speed,
    const int yaw_mode,
    const double yaw_angle,
    const bool use_yaw_angle,
    const std::string & frame_id,
    const bool wait_result = true);
  std::shared_ptr<GoToBehavior> go_to_behavior_;
};

}
}

#endif
