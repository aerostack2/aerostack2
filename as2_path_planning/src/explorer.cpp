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

  planner_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "planner/goal", 10);
  viz_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);

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

  cv::Mat mat = utils::gridToImg(last_occ_grid_);
  cv::Mat edges = cv::Mat(mat.rows, mat.cols, CV_8UC1);
  cv::Canny(mat, edges, 100, 200);

  cv::Mat obstacles = utils::gridToImg(last_occ_grid_, 30, true);

  cv::Point2i origin =
      utils::pointToPixel(drone_pose_, last_occ_grid_.info,
                          last_occ_grid_.header.frame_id, tf_buffer_);
  int iterations =
      std::ceil(0.3 / last_occ_grid_.info.resolution); // ceil to be safe
  // Supposing that drone current cells are obstacles to split frontiers
  cv::Point2i p1 = cv::Point2i(origin.y - iterations, origin.x - iterations);
  cv::Point2i p2 = cv::Point2i(origin.y + iterations, origin.x + iterations);
  // adding drone pose to obstacle mask
  cv::rectangle(obstacles, p1, p2, 0, -1);

  cv::Mat frontiers;
  cv::bitwise_and(obstacles, edges, frontiers);

  // findContours + moments dont work well for 1 pixel lines (polygons)
  // Using connectedComponents instead
  cv::Mat labels, centroids, stats;
  int retVal =
      cv::connectedComponentsWithStats(frontiers, labels, stats, centroids);

  RCLCPP_INFO(this->get_logger(), "Number of labels: %d", retVal);
  // item labeled 0 represents the background label, skip background
  for (int i = 1; i < retVal; i++) {
    if (stats.at<int>(i, cv::CC_STAT_AREA) > FRONTIER_MIN_AREA) {
      std::string name = "label" + std::to_string(i);
      cv::Point2d centroid = centroids.at<cv::Point2d>(i);

      RCLCPP_INFO(this->get_logger(), "Element #%d at (%f %f) with area %d", i,
                  centroid.x, centroid.y, stats.at<int>(i, cv::CC_STAT_AREA));

      auto point =
          utils::pixelToPoint(cv::Point2d(centroid.y, centroid.x),
                              last_occ_grid_.info, last_occ_grid_.header);
      frontier_centroids_.push_back(point);

      cv::Mat mask = cv::Mat::zeros(frontiers.size(), CV_8UC1);
      cv::bitwise_or(mask, (labels == i) * 255, mask);
      std::vector<cv::Point> locations;
      cv::findNonZero(mask, locations);

      visualization_msgs::msg::Marker front_marker = utils::getFrontierMarker(
          name, last_occ_grid_.header.frame_id, this->get_clock()->now(),
          locations, last_occ_grid_.info, last_occ_grid_.header);
      viz_pub_->publish(front_marker);

      visualization_msgs::msg::Marker centroid_marker =
          utils::getPointMarker(name, last_occ_grid_.header.frame_id,
                                this->get_clock()->now(), point.point);
      centroid_marker.color = front_marker.colors[0];
      viz_pub_->publish(centroid_marker);

      // DEBUG VIZ
      // cv::circle(mask, centroids.at<cv::Point2d>(i), 2, 128, -1);
      // cv::namedWindow(name, cv::WINDOW_NORMAL);
      // cv::imshow(name, mask);
    }
  }

  callPlanner();
  // cv::namedWindow("canny", cv::WINDOW_NORMAL);
  // cv::imshow("canny", frontiers);

  cv::waitKey(0);

  // Interesting method?
  // cv::convertScaleAbs(labels, label1);
}

// L2 distance between two 2d points
static double distance(geometry_msgs::msg::Point p1,
                       geometry_msgs::msg::Point p2) {
  double dis = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
  return dis;
}

void Explorer::callPlanner() {
  geometry_msgs::msg::PointStamped closest = frontier_centroids_[0];
  double min_dist = distance(closest.point, goal_.point);

  for (const geometry_msgs::msg::PointStamped &p : frontier_centroids_) {
    closest = distance(p.point, goal_.point) < min_dist ? p : closest;
  }

  RCLCPP_INFO(this->get_logger(), "PLANNER CALL");
  planner_goal_pub_->publish(closest);
}
