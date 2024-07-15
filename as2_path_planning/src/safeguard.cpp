#include <safeguard.hpp>

Safeguard::Safeguard() : Node("safeguard") {
  this->declare_parameter("drone_safety_distance", 1.0); // aprox drone size [m]
  safety_distance_ = this->get_parameter("drone_safety_distance").as_double();
  RCLCPP_INFO(this->get_logger(), "Safety distance: %lf", safety_distance_);

  grid_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/map_server/grid_map", 1,
      std::bind(&Safeguard::gridMapCallback, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(this, this->get_clock(),
                                std::chrono::duration<double>(1.0f / 1.0f),
                                std::bind(&Safeguard::timerCbk, this));
}

void Safeguard::gridMapCallback(
    const grid_map_msgs::msg::GridMap::SharedPtr msg) {
  for (auto layer : msg->layers) {
    if (drones_.find(layer) != drones_.end()) {
      continue;
    }
    // Add to distances_ map
    for (auto drone : drones_) {
      DronePair dp;
      dp.drone1 = drone.first;
      dp.drone2 = layer;
      distances_[dp] = std::numeric_limits<double>::max();
      drones_status_[dp] = SafetyProtocolStatus::OFF;
    }

    // Add to drones_ map
    std::shared_ptr<DroneWatcher> drone_watcher =
        std::make_shared<DroneWatcher>(
            this->get_node_base_interface(), this->get_node_graph_interface(),
            this->get_node_parameters_interface(),
            this->get_node_topics_interface(),
            this->get_node_services_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(), layer);
    drones_[layer] = drone_watcher;
    RCLCPP_INFO(this->get_logger(), "Layer: %s", layer.c_str());
  }
}

void Safeguard::timerCbk() {
  // Update distances
  for (auto drone_pair : distances_) {
    std::string drone1 = drone_pair.first.drone1;
    std::string drone2 = drone_pair.first.drone2;
    double distance =
        utils::distance(drones_[drone1]->drone_pose_.pose.position,
                        drones_[drone2]->drone_pose_.pose.position);
    distances_[drone_pair.first] = distance;
    RCLCPP_INFO(this->get_logger(), "Distance between %s and %s: %lf",
                drone1.c_str(), drone2.c_str(), distance);
  }

  for (auto drone_pair : distances_) {
    std::string drone1 = drone_pair.first.drone1;
    std::string drone2 = drone_pair.first.drone2;
    SafetyProtocolStatus status = drones_status_[drone_pair.first];

    double distance = drone_pair.second;
    if (status == SafetyProtocolStatus::OFF && distance < safety_distance_) {
      RCLCPP_INFO(this->get_logger(), "Distance below safety margin");
      doSafetyProtocol(drone1, drone2);
    } else if (status == SafetyProtocolStatus::ON &&
               distance > safety_distance_ * SAFETY_MARGIN) {
      RCLCPP_INFO(this->get_logger(), "Distance above safety margin");
      endSafetyProtocol(drone1, drone2);
    }
  }
}

void Safeguard::doSafetyProtocol(std::string drone1, std::string drone2) {
  auto drone1_status = drones_[drone1]->traj_gen_status;
  auto drone2_status = drones_[drone2]->traj_gen_status;
  int drone1_priority =
      drones_[drone1]->follow_path_feedback_.feedback.remaining_waypoints;
  int drone2_priority =
      drones_[drone2]->follow_path_feedback_.feedback.remaining_waypoints;

  std::string drone;
  if (drone1_status == as2_msgs::msg::BehaviorStatus::RUNNING &&
      drone2_status == as2_msgs::msg::BehaviorStatus::RUNNING) {
    drone = drone1_priority > drone2_priority ? drone2 : drone1;
  } else if (drone1_status == as2_msgs::msg::BehaviorStatus::IDLE) {
    drone = drone1;
  } else if (drone2_status == as2_msgs::msg::BehaviorStatus::IDLE) {
    drone = drone2;
  } else if (drone1_status == as2_msgs::msg::BehaviorStatus::RUNNING &&
             drone2_status != as2_msgs::msg::BehaviorStatus::RUNNING) {
    drone = drone1;
  } else if (drone1_status != as2_msgs::msg::BehaviorStatus::RUNNING &&
             drone2_status == as2_msgs::msg::BehaviorStatus::RUNNING) {
    drone = drone2;
  } else if (drone1_status != as2_msgs::msg::BehaviorStatus::RUNNING &&
             drone2_status != as2_msgs::msg::BehaviorStatus::RUNNING) {
    return;
  }

  DronePair drone_pair;
  drone_pair.drone1 = drone1;
  drone_pair.drone2 = drone2;
  drones_status_[drone_pair] = SafetyProtocolStatus::ACTIVATING;
  drones_[drone]->avoidanceManeuver();
  RCLCPP_INFO(this->get_logger(), "Drone: %s paused", drone.c_str());
  drones_status_[drone_pair] = SafetyProtocolStatus::ON;
}

void Safeguard::endSafetyProtocol(std::string drone1, std::string drone2) {
  auto drone1_status = drones_[drone1]->traj_gen_status;
  auto drone2_status = drones_[drone2]->traj_gen_status;

  // What if drone has been stopped by other safety protocol?
  std::string drone;
  if (drone1_status == as2_msgs::msg::BehaviorStatus::PAUSED) {
    drone = drone1;
  } else if (drone2_status == as2_msgs::msg::BehaviorStatus::PAUSED) {
    drone = drone2;
  } else {
    RCLCPP_INFO(this->get_logger(), "Drone: %s not paused", drone.c_str());
    return;
  }

  DronePair drone_pair;
  drone_pair.drone1 = drone1;
  drone_pair.drone2 = drone2;
  drones_status_[drone_pair] = SafetyProtocolStatus::DEACTIVATING;
  drones_[drone]->backToFollowPath();
  RCLCPP_INFO(this->get_logger(), "Drone: %s resumed", drone.c_str());
  drones_status_[drone_pair] = SafetyProtocolStatus::OFF;
}
