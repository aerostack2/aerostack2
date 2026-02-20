#include "as2_cpp_api/drone_interface.hpp"

namespace as2
{
namespace as2_cpp_api
{

DroneInterface::DroneInterface(
  const std::string & drone_id,
  bool verbose,
  bool use_sim_time,
  double spin_rate
)
: DroneInterfaceBase(drone_id, verbose, use_sim_time, spin_rate)
{
}

void DroneInterface::initialize()
{
  DroneInterfaceBase::initialize();
  takeoff.initialize(shared_from_this());
  go_to.initialize(shared_from_this());
  land.initialize(shared_from_this());
}

}
}
