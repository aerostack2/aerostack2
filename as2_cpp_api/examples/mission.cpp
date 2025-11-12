#include "as2_cpp_api/drone_interface.hpp"
#include <cmath>

using namespace std::chrono_literals;

const double HEIGHT = 1.0;
const double SPEED = 1.0;
const double LANDING_SPEED = 0.5;

const double RADIUS = 3.0;
const double INITIAL_ANGLE = M_PI / 3;
const std::vector<std::vector<double>> PATH = {
  {RADIUS * std::cos(1 * INITIAL_ANGLE), RADIUS * std::sin(1 * INITIAL_ANGLE), HEIGHT},
  {RADIUS * std::cos(2 * INITIAL_ANGLE), RADIUS * std::sin(2 * INITIAL_ANGLE), HEIGHT},
  {RADIUS * std::cos(3 * INITIAL_ANGLE), RADIUS * std::sin(3 * INITIAL_ANGLE), HEIGHT},
  {RADIUS * std::cos(4 * INITIAL_ANGLE), RADIUS * std::sin(4 * INITIAL_ANGLE), HEIGHT},
  {RADIUS * std::cos(5 * INITIAL_ANGLE), RADIUS * std::sin(5 * INITIAL_ANGLE), HEIGHT},
  {RADIUS * std::cos(6 * INITIAL_ANGLE), RADIUS * std::sin(6 * INITIAL_ANGLE), HEIGHT}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto drone_interface = std::make_shared<as2::as2_cpp_api::DroneInterface>(
    "drone0", false, true,
    20.0);

  RCLCPP_INFO(drone_interface->get_logger(), "Initializing");
  drone_interface->initialize();

  RCLCPP_INFO(drone_interface->get_logger(), "Arming");
  drone_interface->arm();

  RCLCPP_INFO(drone_interface->get_logger(), "Offboarding");
  drone_interface->offboard();

  RCLCPP_INFO(drone_interface->get_logger(), "Taking off");
  if (drone_interface->takeoff(1.0, 0.5, true)) {
    RCLCPP_INFO(drone_interface->get_logger(), "Drone successfully took off!");
  } else {
    RCLCPP_ERROR(drone_interface->get_logger(), "Drone couldn't take off!");
  }

  // we have to wait a little bit here. otherwise, GoToBehavior
  // may reject the goal due to the platform is not flying.
  // probably, this is a sort of bug in the behaviors.
  std::this_thread::sleep_for(500ms);

  RCLCPP_INFO(drone_interface->get_logger(), "Going to the points");
  for (auto point : PATH) {
    RCLCPP_INFO(drone_interface->get_logger(), "Point %f %f %f", point[0], point[1], point[2]);
    if (drone_interface->go_to.go_to_point(point, SPEED, "earth", true)) {
      RCLCPP_ERROR(
        drone_interface->get_logger(), "point (%f %f %f) : SUCCEEDED", point[0], point[1],
        point[2]);
    } else {
      RCLCPP_ERROR(
        drone_interface->get_logger(), "point (%f %f %f) : FAILED", point[0], point[1], point[2]);
    }

    std::this_thread::sleep_for(500ms);
  }

  RCLCPP_INFO(drone_interface->get_logger(), "Going to the points with path facing");
  for (auto point : PATH) {
    RCLCPP_INFO(drone_interface->get_logger(), "Point %f %f %f", point[0], point[1], point[2]);
    if (drone_interface->go_to.go_to_point_path_facing(point, SPEED, "earth", true)) {
      RCLCPP_ERROR(
        drone_interface->get_logger(), "point (%f %f %f) : SUCCEEDED", point[0], point[1],
        point[2]);
    } else {
      RCLCPP_ERROR(
        drone_interface->get_logger(), "point (%f %f %f) : FAILED", point[0], point[1], point[2]);
    }

    std::this_thread::sleep_for(500ms);
  }

  RCLCPP_INFO(drone_interface->get_logger(), "Landing");
  drone_interface->land(LANDING_SPEED);

  RCLCPP_INFO(drone_interface->get_logger(), "Manual");
  drone_interface->manual();

  rclcpp::shutdown();
}
