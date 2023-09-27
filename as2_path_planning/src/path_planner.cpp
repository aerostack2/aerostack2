#include "path_planner.hpp"

PathPlanner::PathPlanner() : Node("path_planner") {
  this->declare_parameter("use_path_optimizer", false);
  use_path_optimizer_ = this->get_parameter("use_path_optimizer").as_bool();

  this->declare_parameter("safety_distance", 1.0); // aprox drone size [m]
  safety_distance_ = this->get_parameter("safety_distance").as_double();

  // TODO: debug mode (bool) to enable visualization

  drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "self_localization/pose", as2_names::topics::self_localization::qos,
      std::bind(&PathPlanner::dronePoseCbk, this, std::placeholders::_1));
  occ_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1,
      std::bind(&PathPlanner::occGridCbk, this, std::placeholders::_1));

  // TODO: change to action, server or even behavior
  planner_goal_sub_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "planner/goal", 10,
          std::bind(&PathPlanner::goalCallback, this, std::placeholders::_1));

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

void PathPlanner::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr point) {
  // World to image transformations
  cv::Point2i goal =
      utils::pointToPixel(*(point), last_occ_grid_.info,
                          last_occ_grid_.header.frame_id, tf_buffer_);

  cv::Point2i origin =
      utils::pointToPixel(drone_pose_, last_occ_grid_.info,
                          last_occ_grid_.header.frame_id, tf_buffer_);

  // Erode obstacles
  cv::Mat mat = utils::gridToImg(last_occ_grid_);
  int iterations = std::ceil(safety_distance_ /
                             last_occ_grid_.info.resolution); // ceil to be safe
  // Supposing that drone current cells are free, mask around drone pose
  cv::Mat mask = cv::Mat::zeros(mat.size(), CV_8UC1);
  cv::Point2i p1 = cv::Point2i(origin.y - iterations, origin.x - iterations);
  cv::Point2i p2 = cv::Point2i(origin.y + iterations, origin.x + iterations);
  cv::rectangle(mask, p1, p2, 255, -1);
  cv::bitwise_or(mat, mask, mat);

  cv::erode(mat, mat, cv::Mat(), cv::Point(-1, -1), iterations);

  planner_algorithm_.setOriginPoint(origin);
  planner_algorithm_.setGoal(goal);
  planner_algorithm_.setOcuppancyGrid(mat);
  auto path = planner_algorithm_.solveGraph();
  if (!path.empty()) {
    path.erase(path.begin()); // popping first element (origin)
  }

  if (use_path_optimizer_) {
    path = path_optimizer::solve(path);
  }
  RCLCPP_INFO(this->get_logger(), "Path size: %ld", path.size());

  // Visualize path
  auto path_marker = utils::getPathMarker(
      last_occ_grid_.header.frame_id, this->get_clock()->now(), path,
      last_occ_grid_.info, last_occ_grid_.header);
  viz_pub_->publish(path_marker);

  // TEST
  mat.at<uchar>(origin.x, origin.y) = 128;
  mat.at<uchar>(goal.x, goal.y) = 128;
  auto test = utils::imgToGrid(mat, last_occ_grid_.header,
                               last_occ_grid_.info.resolution);
  obstacle_grid_pub_->publish(test);
  // END TEST

  // Follow Path behavior
  callFollowPathAction(path_marker.points);
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

  RCLCPP_INFO(this->get_logger(), "Sending goal to FollowPath behavior");
  follow_path_client_->async_send_goal(goal_msg);
}