#ifndef SAFEGUARD_HPP_
#define SAFEGUARD_HPP_

#include "frontier_utils.hpp"
#include <as2_msgs/msg/behavior_status.hpp>
#include <chrono> // std::chrono::duration
#include <drone_watcher.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <map> // std::map
#include <rclcpp/rclcpp.hpp>
#include <tuple> // std::tuple

#define SAFETY_MARGIN 1.2 // 20 %

struct DronePair {
  std::string drone1;
  std::string drone2;

  bool operator==(const DronePair &dp) {
    return (drone1 == dp.drone1 && drone2 == dp.drone2) ||
           (drone1 == dp.drone2 && drone2 == dp.drone1);
  }

  bool operator<(const DronePair &dp) const {
    return std::tie(drone1, drone2) < std::tie(dp.drone1, dp.drone2);
  }

  bool operator>(const DronePair &dp) const {
    return std::tie(drone1, drone2) > std::tie(dp.drone1, dp.drone2);
  }

  bool operator<=(const DronePair &dp) const {
    return std::tie(drone1, drone2) <= std::tie(dp.drone1, dp.drone2);
  }

  bool operator>=(const DronePair &dp) const {
    return std::tie(drone1, drone2) >= std::tie(dp.drone1, dp.drone2);
  }

  bool operator!=(const DronePair &dp) const {
    return std::tie(drone1, drone2) != std::tie(dp.drone1, dp.drone2);
  }
};

enum SafetyProtocolStatus { OFF, ACTIVATING, ON, DEACTIVATING };

class Safeguard : public rclcpp::Node {
public:
  Safeguard();
  ~Safeguard(){};

private:
  std::map<std::string, std::shared_ptr<DroneWatcher>> drones_;
  std::map<DronePair, double> distances_;
  std::map<DronePair, SafetyProtocolStatus> drones_status_;
  double safety_distance_ = 1.0; // [m]

  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  void timerCbk();

  void doSafetyProtocol(std::string drone1, std::string drone2);
  void endSafetyProtocol(std::string drone1, std::string drone2);

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // SAFEGUARD_HPP_