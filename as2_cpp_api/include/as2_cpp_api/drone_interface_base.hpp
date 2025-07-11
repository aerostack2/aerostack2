#ifndef DRONE_INTERFACE_BASE
#define DRONE_INTERFACE_BASE

#include "rclcpp/rclcpp.hpp"
#include <thread>

namespace as2
{
namespace as2_cpp_api
{

class DroneInterfaceBase : public rclcpp::Node
{
public:
  DroneInterfaceBase(
    const std::string & drone_id,
    bool verbose,
    bool use_sim_time,
    double spin_rate,
    const rclcpp::Executor::SharedPtr & executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>());

  ~DroneInterfaceBase();

  virtual void initialize();

  bool arm();
  bool disarm();
  bool offboard();
  bool manual();

private:
  rclcpp::Executor::SharedPtr executor_;
  std::thread spinning_thread_;
};

}
}

#endif
