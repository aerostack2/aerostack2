#include <explorer.hpp>

Explorer::Explorer() : Node("explorer") {
  occ_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1,
      std::bind(&Explorer::occGridCallback, this, std::placeholders::_1));
  drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "self_localization/pose", as2_names::topics::self_localization::qos,
      std::bind(&Explorer::dronePoseCbk, this, std::placeholders::_1));
  debug_point_sub_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "/clicked_point", 10,
          std::bind(&Explorer::clickedPointCallback, this,
                    std::placeholders::_1));

  viz_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);

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

void Explorer::clickedPointCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr point) {
  goal_ = *(point);

  utils::cleanMarkers(viz_pub_, "frontier");

  switch (processGoal(goal_)) {
  case -1:
    // goal in unknown cell
    explore(goal_);
    break;
  case 0:
    // goal in empty cell
    navigateTo(goal_);
    break;
  default:
    // goal in obstacle or out of map, invalid
    return;
    break;
  }
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

int Explorer::processGoal(geometry_msgs::msg::PointStamped goal) {
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
    return cell_value;

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
      std::ceil(SAFETY_DISTANCE / occ_grid.info.resolution); // ceil to be safe
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
  cv::erode(obstacles, obstacles, cv::Mat(), cv::Point(-1, -1), safe_cells);

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
    if (stats.at<int>(i, cv::CC_STAT_AREA) > FRONTIER_MIN_AREA) {
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

// L2 distance between two 2d points
static double distance(geometry_msgs::msg::Point p1,
                       geometry_msgs::msg::Point p2) {
  double dis = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
  return dis;
}

void Explorer::explore(geometry_msgs::msg::PointStamped goal) {
  std::vector<geometry_msgs::msg::PointStamped> centroids = {};
  std::vector<cv::Mat> frontiers = {};
  getFrontiers(last_occ_grid_, centroids, frontiers);

  visualizeFrontiers(centroids, frontiers);

  geometry_msgs::msg::PointStamped closest = centroids[0];
  double min_dist = distance(closest.point, goal.point);
  for (const geometry_msgs::msg::PointStamped &p : centroids) {
    closest = distance(p.point, goal.point) < min_dist ? p : closest;
  }

  navigateTo(closest);
}

void Explorer::navigateTo(geometry_msgs::msg::PointStamped goal) {
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
  navigation_action_client_->async_send_goal(goal_msg, send_goal_options);
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

  if (distance(drone_pose_.pose.position, goal_.point) > 0.5) {
    // try again until final goal reached
    auto goal = std::make_shared<geometry_msgs::msg::PointStamped>(goal_);
    clickedPointCallback(goal);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Navigation ended.");
}
