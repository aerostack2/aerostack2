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

struct Point2iHash {
  std::size_t operator()(const cv::Point2i &point) const {
    // Combine the hash values of x and y using a simple hash function
    return std::hash<int>()(point.x) ^ std::hash<int>()(point.y);
  }
};

static std::vector<cv::Point2i> safeZone(cv::Point2i point, int iterations) {
  std::unordered_set<cv::Point2i, Point2iHash> safe_zone;
  safe_zone.insert({point.y, point.x});
  for (int i = 0; i < iterations; ++i) {
    for (int j = 0; j < iterations; ++j) {
      // Square with size leght = security_cells
      safe_zone.insert({point.y + i, point.x + j});
      safe_zone.insert({point.y - i, point.x - j});
      safe_zone.insert({point.y + i, point.x - j});
      safe_zone.insert({point.y - i, point.x + j});

      // Cross with arm length = security_cells + 1
      safe_zone.insert({point.y + i + 1, point.x});
      safe_zone.insert({point.y - i - 1, point.x});
      safe_zone.insert({point.y, point.x + j + 1});
      safe_zone.insert({point.y, point.x - j - 1});
    }
  }
  std::vector<cv::Point2i> pointVector(safe_zone.begin(), safe_zone.end());
  return pointVector;
}

void Explorer::clickedPointCallback(
    const geometry_msgs::msg::PointStamped point) {
  cv::Mat mat = utils::gridToImg(last_occ_grid_);

  //   cv::blur(mat, mat, cv::Size(5, 5));

  // cv::namedWindow("original", cv::WINDOW_NORMAL);
  // cv::imshow("original", mat);

  cv::Mat edges = cv::Mat(mat.rows, mat.cols, CV_8UC1);
  cv::Canny(mat, edges, 100, 200);

  cv::Mat obstacles = utils::gridToImg(last_occ_grid_, 30, true);

  cv::Point2i origin =
      utils::pointToPixel(drone_pose_, last_occ_grid_.info,
                          last_occ_grid_.header.frame_id, tf_buffer_);
  int iterations =
      std::ceil(0.3 / last_occ_grid_.info.resolution); // ceil to be safe
  // Supposing that drone current cells are obstacles to split frontiers
  auto safe_cells = safeZone(origin, iterations);
  for (const cv::Point2i &p : safe_cells) {
    obstacles.at<uchar>(p) = 0; // obstacles
  }

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
  // cv::namedWindow("canny", cv::WINDOW_NORMAL);
  // cv::imshow("canny", frontiers);

  cv::waitKey(0);

  // Interesting method?
  // cv::convertScaleAbs(labels, label1);
}
