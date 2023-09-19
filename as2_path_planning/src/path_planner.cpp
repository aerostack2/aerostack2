#include "path_planner.hpp"

PathPlanner::PathPlanner() : Node("path_planner") {
  drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "self_localization/pose", as2_names::topics::self_localization::qos,
      std::bind(&PathPlanner::dronePoseCbk, this, std::placeholders::_1));
  occ_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1,
      std::bind(&PathPlanner::occGridCbk, this, std::placeholders::_1));
  debug_point_sub_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "/clicked_point", 10,
          std::bind(&PathPlanner::clickedPointCallback, this,
                    std::placeholders::_1));

  viz_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
  obstacle_grid_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("obstacle_map", 10);

  follow_path_client_ =
      rclcpp_action::create_client<as2_msgs::action::FollowPath>(
          this, as2_names::actions::behaviors::followpath);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void PathPlanner::dronePoseCbk(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  drone_pose_ = *(msg);
}

void PathPlanner::occGridCbk(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  last_occ_grid_ = *(msg);
}

void PathPlanner::clickedPointCallback(
    const geometry_msgs::msg::PointStamped point) {
  //   // TODO: still needed
  //   // Deep copy to avoid modifications on the original map
  //   nav_msgs::msg::OccupancyGrid::SharedPtr occ_grid =
  //       std::make_shared<nav_msgs::msg::OccupancyGrid>();
  //   occ_grid->header = last_occ_grid_.header;
  //   occ_grid->info = last_occ_grid_.info;
  //   occ_grid->data = last_occ_grid_.data;

  // World to image transformations
  cv::Point2i goal = utils::pointToPixel(
      point, last_occ_grid_.info, last_occ_grid_.header.frame_id, tf_buffer_);

  geometry_msgs::msg::PointStamped drone_pose;
  drone_pose.header = drone_pose_.header;
  drone_pose.point = drone_pose_.pose.position;
  cv::Point2i origin =
      utils::pointToPixel(drone_pose, last_occ_grid_.info,
                          last_occ_grid_.header.frame_id, tf_buffer_);

  // Erode obstacles
  cv::Mat mat = utils::gridToImg(last_occ_grid_);
  cv::erode(mat, mat, cv::Mat());
  auto safe_cells = safeZone(origin, last_occ_grid_.info.resolution);
  for (const cv::Point2i &p : safe_cells) {
    mat.at<uchar>(p) = 255; // free
  }

  planner_algorithm_.setOriginPoint(origin);
  planner_algorithm_.setGoal(goal);
  planner_algorithm_.setOcuppancyGrid(mat);
  auto current_path_ = planner_algorithm_.solveGraph();
  RCLCPP_INFO(this->get_logger(), "Path size: %ld", current_path_.size());

  // Visualize path
  auto path_marker = utils::getPathMarker(
      last_occ_grid_.header.frame_id, this->get_clock()->now(), current_path_,
      last_occ_grid_.info, last_occ_grid_.header);
  viz_pub_->publish(path_marker);

  // TEST
  mat.at<uchar>(origin.x - 1, origin.y - 1) = 128;
  mat.at<uchar>(goal.x - 1, goal.y - 1) = 128;
  auto test = utils::imgToGrid(mat, last_occ_grid_.header,
                               last_occ_grid_.info.resolution);
  obstacle_grid_pub_->publish(test);
  // END TEST

  // Follow Path behavior
  callFollowPathAction(path_marker.points);
}

struct Point2iHash {
  std::size_t operator()(const cv::Point2i &point) const {
    // Combine the hash values of x and y using a simple hash function
    return std::hash<int>()(point.x) ^ std::hash<int>()(point.y);
  }
};

std::vector<cv::Point2i> PathPlanner::safeZone(cv::Point2i point,
                                               double grid_resolution) {
  float security_distance = 0.5;
  int security_cells = std::round(security_distance / grid_resolution);

  std::unordered_set<cv::Point2i, Point2iHash> safe_zone;
  for (int i = 0; i < security_cells; ++i) {
    for (int j = 0; j < security_cells; ++j) {
      safe_zone.insert({point.y + i - 1, point.x + j - 1});
      safe_zone.insert({point.y - i - 1, point.x - j - 1});
      safe_zone.insert({point.y + i - 1, point.x - j - 1});
      safe_zone.insert({point.y - i - 1, point.x + j - 1});
    }
  }
  std::vector<cv::Point2i> pointVector(safe_zone.begin(), safe_zone.end());
  return pointVector;
}

void PathPlanner::callFollowPathAction(
    std::vector<geometry_msgs::msg::Point> points) {
  if (!this->follow_path_client_->wait_for_action_server(
          std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    return;
  }

  auto goal_msg = as2_msgs::action::FollowPath::Goal();
  goal_msg.header.frame_id = "earth";
  goal_msg.header.stamp = this->get_clock()->now();
  goal_msg.yaw.mode = as2_msgs::msg::YawMode::PATH_FACING;
  goal_msg.max_speed = 1.0;
  int i = 0;
  for (auto &p : points) {
    as2_msgs::msg::PoseWithID pid = as2_msgs::msg::PoseWithID();
    pid.id = std::to_string(i);
    pid.pose.position = p;
    pid.pose.position.z = 1.0;
    goal_msg.path.push_back(pid);
    i++;
  }

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  follow_path_client_->async_send_goal(goal_msg);
}