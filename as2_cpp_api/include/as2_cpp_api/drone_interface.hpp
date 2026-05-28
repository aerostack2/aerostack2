#ifndef DRONE_INTERFACE
#define DRONE_INTERFACE

#include "as2_cpp_api/drone_interface_base.hpp"
#include "as2_cpp_api/modules/takeoff_module.hpp"
#include "as2_cpp_api/modules/go_to_module.hpp"
#include "as2_cpp_api/modules/land_module.hpp"

namespace as2
{
namespace as2_cpp_api
{

class DroneInterface : public DroneInterfaceBase
{
public:
  DroneInterface(
    const std::string & drone_id,
    bool verbose,
    bool use_sim_time,
    double spin_rate
  );

  ~DroneInterface() = default;

  void initialize() override;

  TakeoffModule takeoff;
  GoToModule go_to;
  LandModule land;
};

}
}

#endif
