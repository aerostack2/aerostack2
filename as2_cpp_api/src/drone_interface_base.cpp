#include "as2_cpp_api/drone_interface_base.hpp"
#include "as2_cpp_api/service_clients/arming.hpp"
#include "as2_cpp_api/service_clients/offboard.hpp"

namespace as2
{
namespace as2_cpp_api
{

DroneInterfaceBase::DroneInterfaceBase(
  const std::string & drone_id,
  bool verbose,
  bool use_sim_time,
  double spin_rate,
  const rclcpp::Executor::SharedPtr & executor)
: rclcpp::Node(drone_id + "_interface", drone_id), executor_(executor)
{
}

void DroneInterfaceBase::initialize()
{
  rclcpp::Parameter param_use_sim_time("use_sim_time", true);
  this->set_parameter(param_use_sim_time);

  executor_->add_node(shared_from_this());

  spinning_thread_ = std::thread(
    [this]() {
      executor_->spin();
    });
}

DroneInterfaceBase::~DroneInterfaceBase()
{
  executor_->cancel();
  if (spinning_thread_.joinable()) {
    spinning_thread_.join();
  }
}

bool DroneInterfaceBase::arm()
{
  auto arm_handler = Arm(shared_from_this());
  return arm_handler();
}

bool DroneInterfaceBase::disarm()
{
  auto disarm_handler = Disarm(shared_from_this());
  return disarm_handler();
}

bool DroneInterfaceBase::offboard()
{
  auto offboard_handler = Offboard(shared_from_this());
  return offboard_handler();
}

bool DroneInterfaceBase::manual()
{
  auto manual_handler = Manual(shared_from_this());
  return manual_handler();
}

}
}
