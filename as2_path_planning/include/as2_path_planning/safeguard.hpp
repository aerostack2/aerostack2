#ifndef SAFEGUARD_HPP_
#define SAFEGUARD_HPP_

#include "frontier_utils.hpp"
#include <as2_msgs/msg/behavior_status.hpp>
#include <chrono> // std::chrono::duration
#include <drone_watcher.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tuple>         // std::tuple
#include <unordered_map> // std::unordered_map

#define SAFETY_MARGIN 1.2 // 20 %

struct DronePair {
  std::string drone1;
  std::string drone2;
};

bool operator==(const DronePair &lhs, const DronePair &rhs);

template <> struct std::hash<DronePair> {
  std::size_t operator()(const DronePair &dp) const noexcept {
    std::size_t h1 = std::hash<std::string>{}(dp.drone1);
    std::size_t h2 = std::hash<std::string>{}(dp.drone2);
    return h1 ^ (h2 << 1);
  }
};

enum SafetyProtocolStatus { OFF, ACTIVATING, ON, DEACTIVATING };

class Safeguard : public rclcpp::Node {
public:
  Safeguard();
  ~Safeguard(){};

private:
  std::unordered_map<std::string, std::shared_ptr<DroneWatcher>> drones_;
  std::unordered_map<DronePair, double> distances_;
  std::unordered_map<DronePair, SafetyProtocolStatus> drones_status_;
  double safety_distance_ = 1.0; // [m]

  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  void timerCbk();

  void doSafetyProtocol(std::string drone1, std::string drone2);
  void endSafetyProtocol(std::string drone1, std::string drone2);

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // SAFEGUARD_HPP_