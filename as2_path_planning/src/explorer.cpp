#include <explorer.hpp>

Explorer::Explorer() : Node("explorer") {
  this->declare_parameter("frontier_min_area", 1);
  frontier_min_area_ = this->get_parameter("frontier_min_area").as_int();

  this->declare_parameter("safety_distance", 1.0); // aprox drone size [m]
  safety_distance_ = this->get_parameter("safety_distance").as_double();

  this->declare_parameter("reached_dist_thresh", 0.5);
  reached_dist_thresh_ = this->get_parameter("reached_dist_thresh").as_double();

  occ_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1,
      std::bind(&Explorer::occGridCallback, this, std::placeholders::_1));
  drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "self_localization/pose", as2_names::topics::self_localization::qos,
      std::bind(&Explorer::dronePoseCbk, this, std::placeholders::_1));

  rclcpp::SubscriptionOptions options;
  cbk_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = cbk_group_;
  debug_point_sub_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "/clicked_point", 10,
          std::bind(&Explorer::clickedPointCallback, this,
                    std::placeholders::_1),
          options);

  viz_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);

  start_explore_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "start_exploration",
      std::bind(&Explorer::startExplorationCbk, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cbk_group_);

  navigation_action_client_ =
      rclcpp_action::create_client<NavigateToPoint>(this, "navigate_to_point");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void Explorer::occGridCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  last_occ_grid_ = *(msg);
}

void Explorer::dronePoseCbk(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  drone_pose_ = *(msg);
}

// L2 distance between two 2d points
static double distance(geometry_msgs::msg::Point p1,
                       geometry_msgs::msg::Point p2) {
  double dis = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
  return dis;
}

void Explorer::startExplorationCbk(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (request->data) {
    geometry_msgs::msg::PointStamped goal;

    RCLCPP_INFO(this->get_logger(), "Starting exploration.");
    int result = 0;
    while (result == 0) {
      goal.header = drone_pose_.header;
      goal.point = drone_pose_.pose.position;
      result = explore(goal);
      // while able to navigate to a frontier, keep exploring
    }

    if (result == 0) {
      RCLCPP_INFO(this->get_logger(), "Exploration ended succesfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Exploration failed.");
    }

  } else {
    RCLCPP_INFO(this->get_logger(), "Stopping exploration.");
  }
  response->success = true;
}

void Explorer::clickedPointCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr point) {
  int result = 0;
  while (distance(drone_pose_.pose.position, point->point) >
             reached_dist_thresh_ &&
         result == 0) {
    switch (processGoal(*point)) {
    case -1:
      // goal in unknown cell
      result = explore(*point);
      break;
    case 0:
      // goal in empty cell
      result = navigateTo(*point);
      break;
    default:
      // goal in obstacle or out of map, invalid
      return;
      break;
    }
  }

  if (result != 0) {
    RCLCPP_ERROR(this->get_logger(), "Goal unreachable.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Goal reached.");
}

void Explorer::visualizeFrontiers(
    const std::vector<geometry_msgs::msg::PointStamped> &centroids,
    const std::vector<cv::Mat> &frontiers) {
  for (int i = 0; i < centroids.size(); i++) {
    std::string name = "frontier_" + std::to_string(i);
    cv::Mat frontier = frontiers[i];
    geometry_msgs::msg::PointStamped centroid = centroids[i];

    std::vector<cv::Point> locations;
    cv::findNonZero(frontier, locations);

    std_msgs::msg::Header header = last_occ_grid_.header;
    header.stamp = this->get_clock()->now();
    visualization_msgs::msg::Marker front_marker =
        utils::getFrontierMarker(i, locations, last_occ_grid_.info, header);
    viz_pub_->publish(front_marker);

    visualization_msgs::msg::Marker centroid_marker =
        utils::getPointMarker("frontier", 1000 + i, header, centroid.point);
    centroid_marker.color = front_marker.colors[0];
    viz_pub_->publish(centroid_marker);
  }
}

// TODO: make static?
int Explorer::processGoal(geometry_msgs::msg::PointStamped goal) {
  cv::Mat map = utils::gridToImg(last_occ_grid_);
  // Eroding map to avoid frontiers on map borders
  int safe_cells = std::ceil(safety_distance_ /
                             last_occ_grid_.info.resolution); // ceil to be safe
  cv::erode(map, map, cv::Mat(), cv::Point(-1, -1), safe_cells);

  nav_msgs::msg::OccupancyGrid eroded_grid = utils::imgToGrid(
      map, last_occ_grid_.header, last_occ_grid_.info.resolution);

  std::vector<int> cell_goal =
      utils::pointToCell(goal, last_occ_grid_.info, "earth", tf_buffer_);

  if (cell_goal[0] >= 0 && cell_goal[0] < last_occ_grid_.info.width &&
      cell_goal[1] >= 0 && cell_goal[1] < last_occ_grid_.info.height) {
    int cell_index = cell_goal[1] * last_occ_grid_.info.width + cell_goal[0];
    int cell_value = last_occ_grid_.data[cell_index];
    if (cell_value > 0) {
      RCLCPP_ERROR(this->get_logger(), "Goal in obstacle.");
      return 1;
    }
    // free (0) or unknown (-1) cell
    return eroded_grid.data[cell_index] > 0 ? -1 : 0;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Goal out of map.");
    return 1;
  }
}

void Explorer::getFrontiers(
    const nav_msgs::msg::OccupancyGrid &occ_grid,
    std::vector<geometry_msgs::msg::PointStamped> &centroidsOutput,
    std::vector<cv::Mat> &frontiersOutput) {
  // Get edges of binary map
  cv::Mat map = utils::gridToImg(occ_grid);
  // Eroding map to avoid frontiers on map borders
  int safe_cells =
      std::ceil(safety_distance_ / occ_grid.info.resolution); // ceil to be safe
  cv::erode(map, map, cv::Mat(), cv::Point(-1, -1), safe_cells);

  cv::Mat edges = cv::Mat(map.rows, map.cols, CV_8UC1);
  cv::Canny(map, edges, 100, 200);

  // Obstacle map to apply mask on edges
  cv::Mat obstacles = utils::gridToImg(occ_grid, 30, true);

  cv::Point2i origin = utils::pointToPixel(
      drone_pose_, occ_grid.info, occ_grid.header.frame_id, tf_buffer_);
  // Supposing that drone current cells are obstacles to split frontiers
  cv::Point2i p1 = cv::Point2i(origin.y - safe_cells, origin.x - safe_cells);
  cv::Point2i p2 = cv::Point2i(origin.y + safe_cells, origin.x + safe_cells);
  // adding drone pose to obstacle mask
  cv::rectangle(obstacles, p1, p2, 0, -1);
  // eroding obstacles to avoid frontier centroid on impassable cells
  cv::erode(obstacles, obstacles, cv::Mat(3, 3, CV_8UC1), cv::Point(-1, -1),
            safe_cells + 1);

  cv::Mat frontiers;
  cv::bitwise_and(obstacles, edges, frontiers);

  // findContours + moments dont work well for 1 pixel lines (polygons)
  // Using connectedComponents instead
  cv::Mat labels, centroidsPx, stats;
  int retVal =
      cv::connectedComponentsWithStats(frontiers, labels, stats, centroidsPx);

  // item labeled 0 represents the background label, skip background
  for (int i = 1; i < retVal; i++) {
    // filtering frontiers
    if (stats.at<int>(i, cv::CC_STAT_AREA) > frontier_min_area_) {
      auto point = utils::pixelToPoint(centroidsPx.at<double>(i, 1),
                                       centroidsPx.at<double>(i, 0),
                                       occ_grid.info, occ_grid.header);
      centroidsOutput.push_back(point);

      cv::Mat mask = cv::Mat::zeros(frontiers.size(), CV_8UC1);
      cv::bitwise_or(mask, (labels == i) * 255, mask);
      frontiersOutput.push_back(mask);
    }
  }
}

std::vector<geometry_msgs::msg::PointStamped> Explorer::filterCentroids(
    const nav_msgs::msg::OccupancyGrid &occ_grid,
    const std::vector<geometry_msgs::msg::PointStamped> &centroids) {
  std::vector<geometry_msgs::msg::PointStamped> filtered_centroids = {};
  for (const geometry_msgs::msg::PointStamped &p : centroids) {
    std::vector<int> cell =
        utils::pointToCell(p, occ_grid.info, "earth", tf_buffer_);
    int cell_index = cell[1] * occ_grid.info.width + cell[0];
    int cell_value = occ_grid.data[cell_index];
    if (cell_value == 0) {
      filtered_centroids.push_back(p);
    }
  }
  return filtered_centroids;
}

/*
 * Navigate to the closest reachable frontier centroid to the given point.
 * Returns:
 *    0: navigation ended successfully
 *    1: navigation failed
 *   -1: navigation rejected by server
 *   -2: no frontiers found
 */
int Explorer::explore(geometry_msgs::msg::PointStamped goal) {
  utils::cleanMarkers(viz_pub_, "frontier");

  std::vector<geometry_msgs::msg::PointStamped> centroids = {};
  std::vector<cv::Mat> frontiers = {};
  getFrontiers(last_occ_grid_, centroids, frontiers);

  visualizeFrontiers(centroids, frontiers);
  // TODO: this is temporal, remove when frontier detection is improved
  centroids = filterCentroids(last_occ_grid_, centroids);
  if (centroids.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "No frontiers found.");
    return -2;
  }

  int result;
  geometry_msgs::msg::PointStamped closest;
  do {
    closest = centroids[0];
    double min_dist = distance(closest.point, goal.point);
    for (const geometry_msgs::msg::PointStamped &p : centroids) {
      closest = distance(p.point, goal.point) < min_dist ? p : closest;
    }

    result = navigateTo(closest);
    // if navigation failed, remove closest frontier and try with next closest
    centroids.erase(std::remove(centroids.begin(), centroids.end(), closest),
                    centroids.end());
  } while (result != 0 && centroids.size() > 0);

  return result;
}

/*
 * Call path_planner to navigate to the given point. Be careful, sync method.
 * Returns:
 *    0: navigation ended successfully
 *    1: navigation aborted or canceled
 *   -1: navigation rejected by server
 */
int Explorer::navigateTo(geometry_msgs::msg::PointStamped goal) {
  auto goal_msg = NavigateToPoint::Goal();
  goal_msg.point = goal;

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPoint>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&Explorer::navigationResponseCbk, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&Explorer::navigationFeedbackCbk, this, std::placeholders::_1,
                std::placeholders::_2);
  send_goal_options.result_callback =
      std::bind(&Explorer::navigationResultCbk, this, std::placeholders::_1);

  auto goal_handle_future =
      navigation_action_client_->async_send_goal(goal_msg, send_goal_options);

  goal_handle_future.wait(); // TODO: might block forever
  auto goal_handle = goal_handle_future.get();
  if (goal_handle == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Navigation rejected from server");
    return -1;
  }

  // Waiting for navigation to end
  bool nav_end = false;
  while (!nav_end) {
    switch (goal_handle->get_status()) {
    case rclcpp_action::GoalStatus::STATUS_ABORTED:
      return 1;
      break;
    case rclcpp_action::GoalStatus::STATUS_CANCELED:
      return 1;
      break;
    case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
      nav_end = true;
      break;
    // case rclcpp_action::GoalStatus::STATUS_ACCEPTED:
    //   break;
    // case rclcpp_action::GoalStatus::STATUS_CANCELING:
    //   break;
    // case rclcpp_action::GoalStatus::STATUS_EXECUTING:
    //   break;
    // case rclcpp_action::GoalStatus::STATUS_UNKNOWN:
    //   break;
    default:
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}

void Explorer::navigationResponseCbk(
    const GoalHandleNavigateToPoint::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Navigation request rejected by server.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Navigation started.");
  }
}

void Explorer::navigationFeedbackCbk(
    GoalHandleNavigateToPoint::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPoint::Feedback> feedback) {
  RCLCPP_DEBUG(this->get_logger(), "%lf %lf",
               feedback->current_pose.pose.position.x,
               feedback->current_pose.pose.position.y);
}

void Explorer::navigationResultCbk(
    const GoalHandleNavigateToPoint::WrappedResult &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Navigation was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Navigation was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Navigation ended successfully.");
}
