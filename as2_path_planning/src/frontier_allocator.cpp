#include "frontier_allocator.hpp"

FrontierAllocator::FrontierAllocator() : Node("frontier_allocator") {
  this->declare_parameter("safety_distance", 0.25); // [m]
  safety_distance_ = this->get_parameter("safety_distance").as_double();

  this->declare_parameter("frontier_min_area", 15); // [cells]
  frontier_min_area_ = this->get_parameter("frontier_min_area").as_int();

  this->declare_parameter("frontier_max_area", 25); // [cells]
  frontier_max_area_ = this->get_parameter("frontier_max_area").as_int();

  occ_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map_server/map_filtered", 1,
      std::bind(&FrontierAllocator::occGridCallback, this,
                std::placeholders::_1));
  allocate_frontier_srv_ = create_service<as2_msgs::srv::AllocateFrontier>(
      "allocate_frontier",
      std::bind(&FrontierAllocator::allocateFrontierCbk, this,
                std::placeholders::_1, std::placeholders::_2));

  viz_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("marker", 50);
}

void FrontierAllocator::occGridCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  last_occ_grid_ = msg;
};

void FrontierAllocator::allocateFrontierCbk(
    const as2_msgs::srv::AllocateFrontier::Request::SharedPtr request,
    as2_msgs::srv::AllocateFrontier::Response::SharedPtr response) {
  if (last_occ_grid_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "No map received yet");
    response->success = false;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received request from %s",
              request->explorer_id.c_str());

  utils::cleanMarkers(viz_pub_, "frontier");

  std::vector<Frontier> frontiers = {};
  getFrontiers(*last_occ_grid_.get(), frontiers);

  visualizeFrontiers(frontiers);

  if (frontiers.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "No frontiers found.");
    response->success = false;
    return;
  }

  // TODO: loop when two closest frontiers are unreacheable
  Frontier next = explorationHeuristic(request->explorer_pose, frontiers);
  if (next.area == 0) {
    RCLCPP_ERROR(this->get_logger(), "No frontier available.");
    response->success = true;
    response->frontier = geometry_msgs::msg::PointStamped();
    response->frontier.header.frame_id = "none";
    return;
  }

  allocated_frontiers_[request->explorer_id] = next;
  response->frontier = next.goal;
  response->success = true;
};

void FrontierAllocator::getFrontiers(
    const nav_msgs::msg::OccupancyGrid &occ_grid,
    std::vector<Frontier> &frontiersOutput) {
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
  // eroding obstacles to avoid frontier centroid on impassable cells
  cv::erode(obstacles, obstacles, cv::Mat(), cv::Point(-1, -1), safe_cells + 1);

  cv::Mat frontiers_mat;
  cv::bitwise_and(obstacles, edges, frontiers_mat);

  // Labels each connected component of the frontier (each frontier will have
  // and unique value associated to all of its pixels)
  cv::Mat labels, centroidsPx, stats;
  int retVal = cv::connectedComponentsWithStats(frontiers_mat, labels, stats,
                                                centroidsPx);
  // findContours + moments dont work well for 1 pixel lines (polygons)
  // Using connectedComponents instead

  // item labeled 0 represents the background label, skip background
  for (int i = 1; i < retVal; i++) {
    // filtering frontiers
    if (stats.at<int>(i, cv::CC_STAT_AREA) < frontier_min_area_) {
      continue;
    }

    cv::Mat mask = cv::Mat::zeros(frontiers_mat.size(), CV_8UC1);
    cv::bitwise_or(mask, (labels == i) * 255, mask);

    int n_parts = (int)std::floor(stats.at<int>(i, cv::CC_STAT_AREA) /
                                  frontier_max_area_);
    if (n_parts > 1) {
      // splitFrontier(mask, n_parts, centroidsOutput, frontiersOutput);
      splitFrontierSnake(mask, n_parts, frontiersOutput);
      continue;
    }

    std::vector<cv::Point2i> pixelLocations;
    cv::findNonZero(mask, pixelLocations);
    auto goal = utils::closest(pixelLocations, centroidsPx.at<double>(i, 0),
                               centroidsPx.at<double>(i, 1));
    Frontier frontier;
    frontier.goal = utils::pixelToPoint(goal->y, goal->x, last_occ_grid_->info,
                                        last_occ_grid_->header);
    frontier.centroid = utils::pixelToPoint(centroidsPx.at<double>(i, 1),
                                            centroidsPx.at<double>(i, 0),
                                            occ_grid.info, occ_grid.header);
    frontier.area = stats.at<int>(i, cv::CC_STAT_AREA);
    frontier.orientation = 0.0; // TODO
    frontier.labeled_mat = mask;
    frontiersOutput.push_back(frontier);
  }
}

void FrontierAllocator::splitFrontier(const cv::Mat &frontier, int n_parts,
                                      std::vector<Frontier> &frontiersOutput) {
  // Locate the non-zero pixel values
  std::vector<cv::Point2d> pixelLocations;
  cv::findNonZero(frontier, pixelLocations);

  std::vector<double> line; // (vx, vy, x0, y0), v is normalized
  cv::fitLine(pixelLocations, line, cv::DIST_L2, 0, 0.01, 0.01);
  cv::Point2f x_axis(1, 0);
  float angle = std::acos((x_axis.x * line[0] + x_axis.y * line[1])); // rad

  // rotate points
  cv::Point2i origin(frontier.size().width / 2, frontier.size().height / 2);
  std::vector<cv::Point2d> rotated_pts =
      utils::rotatePoints(pixelLocations, origin, -angle);

  // TODO: apply transformation when sorting instead of rotating and rotating
  // back?

  // sort points
  std::sort(rotated_pts.begin(), rotated_pts.end(), utils::pt_comp);

  // split in n_parts
  int n = rotated_pts.size();
  // https://stackoverflow.com/questions/62032583/division-round-up-in-c
  int size_max = (n + (n_parts - 1)) / n_parts;
  std::vector<std::vector<cv::Point2d>> split_pts;
  for (int i = 0; i < n; i += size_max) {
    int iend = i + size_max > n ? n : i + size_max;
    split_pts.emplace_back(std::vector<cv::Point2d>(
        rotated_pts.begin() + i, rotated_pts.begin() + iend));
  }

  // rotate back points
  for (std::vector<cv::Point2d> pts : split_pts) {
    std::vector<cv::Point2d> back_rotated_pts =
        utils::rotatePoints(pts, origin, angle);
    cv::Mat mask = cv::Mat::zeros(frontier.size(), CV_8UC1);
    double total_x = 0, total_y = 0;
    for (cv::Point pt : back_rotated_pts) {
      mask.at<uchar>(pt.y, pt.x) = 255;
      total_x += pt.x;
      total_y += pt.y;
    }
    cv::Point2i px_centroid(total_y / pts.size(), total_x / pts.size());
    Frontier front;
    front.labeled_mat = mask;
    front.area = pts.size();
    front.orientation = 0.0; // TODO
    front.centroid = utils::pixelToPoint(px_centroid, last_occ_grid_->info,
                                         last_occ_grid_->header);
    frontiersOutput.push_back(front);
  }
}

/* Looking for endpoints, which should only have one neighbor. Then sorting
from one endpoint to the other based on my neightbor should have minimum
distance to me. Then array splitting in n parts */
void FrontierAllocator::splitFrontierSnake(
    const cv::Mat &frontier, int n_parts,
    std::vector<Frontier> &frontiersOutput) {
  cv::Mat scaled = cv::Mat::zeros(frontier.size(), CV_8UC1);
  // Using 100 -> then endpoint value will be 100*2 (neighbors + itself)
  scaled.setTo(100, frontier == 255);

  cv::Mat filtered = cv::Mat(frontier.rows, frontier.cols, CV_8UC1);
  cv::filter2D(scaled, filtered, -1, cv::Mat::ones(3, 3, CV_8UC1));

  // looking for endpoints, getting min value. Masking to only keep pixels that
  // where originaly in the frontier
  double min_val, max_val;
  cv::Point2i min_loc, max_loc;
  cv::minMaxLoc(filtered, &min_val, &max_val, &min_loc, &max_loc, frontier);

  // Locate the non-zero pixel values
  std::vector<cv::Point2i> pixelLocations;
  cv::findNonZero(frontier, pixelLocations);

  if (min_val == max_val) {
    min_loc = pixelLocations[0];
  }

  if (std::find(pixelLocations.begin(), pixelLocations.end(), min_loc) ==
      pixelLocations.end()) {
    pixelLocations.insert(pixelLocations.begin(), min_loc);
  }

  // sorting points
  std::vector<cv::Point2i> sorted_pts = utils::snakeSort(
      pixelLocations,
      std::find(pixelLocations.begin(), pixelLocations.end(), min_loc));

  // split in n_parts
  int n = sorted_pts.size();
  // https://stackoverflow.com/questions/62032583/division-round-up-in-c
  int size_max = (n + (n_parts - 1)) / n_parts;
  std::vector<std::vector<cv::Point2i>> split_pts;
  for (int i = 0; i < n; i += size_max) {
    int iend = i + size_max > n ? n : i + size_max;
    split_pts.emplace_back(std::vector<cv::Point2i>(sorted_pts.begin() + i,
                                                    sorted_pts.begin() + iend));
  }

  // get centroids of tokens
  for (const std::vector<cv::Point2i> &pts : split_pts) {
    cv::Mat mask = cv::Mat::zeros(frontier.size(), CV_8UC1);
    double total_x = 0, total_y = 0;
    for (const cv::Point &pt : pts) {
      mask.at<uchar>(pt.y, pt.x) = 255;
      total_x += pt.x;
      total_y += pt.y;
    }
    cv::Point2i px_centroid(total_x / pts.size(), total_y / pts.size());
    auto goal = utils::closest(pts, px_centroid);
    Frontier frontier;
    frontier.goal = utils::pixelToPoint(goal->y, goal->x, last_occ_grid_->info,
                                        last_occ_grid_->header);
    frontier.centroid =
        utils::pixelToPoint(px_centroid.y, px_centroid.x, last_occ_grid_->info,
                            last_occ_grid_->header);
    frontier.area = pts.size();
    frontier.orientation = 0.0; // TODO
    frontier.labeled_mat = mask;
    frontiersOutput.push_back(frontier);
  }
}

void FrontierAllocator::visualizeFrontiers(
    const std::vector<Frontier> &frontiers) {
  for (int i = 0; i < frontiers.size(); i++) {
    std::string name = "frontier_" + std::to_string(i);
    cv::Mat frontier_mask = frontiers[i].labeled_mat;
    geometry_msgs::msg::PointStamped centroid = frontiers[i].centroid;

    std::vector<cv::Point> locations;
    cv::findNonZero(frontier_mask, locations);

    std_msgs::msg::Header header = last_occ_grid_->header;
    header.stamp = this->get_clock()->now();
    visualization_msgs::msg::Marker front_marker =
        utils::getFrontierMarker(i, locations, last_occ_grid_->info, header);
    viz_pub_->publish(front_marker);

    visualization_msgs::msg::Marker centroid_marker =
        utils::getPointMarker("frontier", 1000 + i, header, centroid.point);
    centroid_marker.color = front_marker.colors[0];
    viz_pub_->publish(centroid_marker);

    visualization_msgs::msg::Marker label = utils::getTextMarker(
        "frontier", 2000 + i, header, centroid.point, std::to_string(i));
    viz_pub_->publish(label);
  }
}

Frontier FrontierAllocator::explorationHeuristic(
    const geometry_msgs::msg::PointStamped &goal,
    const std::vector<Frontier> &frontiers) {
  auto available_frontiers = frontiers;
  // Avoid current asigned frontiers
  for (auto const &[key, value] : allocated_frontiers_) {
    available_frontiers.erase(std::remove(available_frontiers.begin(),
                                          available_frontiers.end(), value),
                              available_frontiers.end());
  }

  if (available_frontiers.size() == 0) {
    return Frontier();
  }
  return getCloserFrontier(goal, available_frontiers);
}

Frontier FrontierAllocator::explorationHeuristic(
    const geometry_msgs::msg::PoseStamped &goal,
    const std::vector<Frontier> &frontiers) {
  geometry_msgs::msg::PointStamped goal_point;
  goal_point.header = goal.header;
  goal_point.point = goal.pose.position;
  return explorationHeuristic(goal_point, frontiers);
}

// Simple Heuristic: Closes centroid to goal
Frontier FrontierAllocator::getCloserFrontier(
    const geometry_msgs::msg::PointStamped &goal,
    const std::vector<Frontier> &frontiers) {
  Frontier closest = frontiers[0];
  double min_dist = utils::distance(closest.centroid.point, goal.point);
  for (const Frontier &f : frontiers) {
    double dist = utils::distance(f.centroid.point, goal.point);
    closest = dist < min_dist ? f : closest;
    min_dist = dist < min_dist ? dist : min_dist;
  }
  return closest;
}