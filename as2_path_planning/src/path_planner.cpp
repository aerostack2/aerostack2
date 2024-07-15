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
      "/map_server/map_filtered", 1,
      std::bind(&PathPlanner::occGridCbk, this, std::placeholders::_1));

  viz_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
  obstacle_grid_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("obstacle_map", 10);

  navigation_action_server_ = rclcpp_action::create_server<NavigateToPoint>(
      this, "navigate_to_point",
      std::bind(&PathPlanner::navigationGoalCbk, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&PathPlanner::navigationCancelCbk, this, std::placeholders::_1),
      std::bind(&PathPlanner::navigationAcceptedCbk, this,
                std::placeholders::_1));

  follow_path_client_ = rclcpp_action::create_client<FollowPath>(
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

rclcpp_action::GoalResponse PathPlanner::navigationGoalCbk(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const NavigateToPoint::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request with point (%f, %f)",
              goal->point.point.x, goal->point.point.y);

  // World to image transformations
  cv::Point2i goal_px =
      utils::pointToPixel(goal->point, last_occ_grid_.info,
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

  // Visualize obstacle map
  mat.at<uchar>(origin.x, origin.y) = 128;
  mat.at<uchar>(goal_px.x, goal_px.y) = 128;
  auto test = utils::imgToGrid(mat, last_occ_grid_.header,
                               last_occ_grid_.info.resolution);
  obstacle_grid_pub_->publish(test);

  planner_algorithm_.setOriginPoint(origin);
  planner_algorithm_.setGoal(goal_px);
  planner_algorithm_.setOcuppancyGrid(mat);
  auto path = planner_algorithm_.solveGraph();
  if (!path.empty()) {
    path.erase(path.begin()); // popping first element (origin)
  } else {
    RCLCPP_ERROR(this->get_logger(), "Path to goal not found");
    int cell_value = mat.at<uchar>(origin.x, origin.y);
    if (cell_value > 0) {
      RCLCPP_ERROR(this->get_logger(), "Origin is unreachable.");
    }
    cell_value = mat.at<uchar>(goal_px.x, goal_px.y);
    if (cell_value > 0) {
      RCLCPP_ERROR(this->get_logger(), "Goal is unreachable.");
    }
    return rclcpp_action::GoalResponse::REJECT;
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

  // TODO: split path generator from visualization
  path_ = path_marker.points;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathPlanner::navigationCancelCbk(
    const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  // TODO: cancel only the goal started from navigation. Behaviors only accepts
  // one goal simultaneously, don't have to worry about
  follow_path_client_->async_cancel_all_goals();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PathPlanner::navigationAcceptedCbk(
    const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }

  execution_thread_ = std::thread(
      std::bind(&PathPlanner::navigateToPoint, this, std::placeholders::_1),
      goal_handle);
}

void PathPlanner::navigateToPoint(
    const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
  navigation_goal_handle_ = goal_handle;

  // Call Follow Path behavior
  if (!this->follow_path_client_->wait_for_action_server(
          std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Follow Path Action server not available "
                                     "after waiting. Aborting navigation.");
    auto nav_result = std::make_shared<NavigateToPoint::Result>();
    nav_result->success = false;
    navigation_goal_handle_->abort(nav_result);
    return;
  }

  auto goal_msg = FollowPath::Goal();
  goal_msg.header.frame_id = "earth";
  goal_msg.header.stamp = this->get_clock()->now();
  goal_msg.yaw = navigation_goal_handle_->get_goal()->yaw;
  goal_msg.max_speed = navigation_goal_handle_->get_goal()->navigation_speed;
  int i = 0;
  for (auto &p : path_) {
    as2_msgs::msg::PoseWithID pid = as2_msgs::msg::PoseWithID();
    pid.id = std::to_string(i);
    pid.pose.position = p;
    pid.pose.position.z = 1.0;
    goal_msg.path.push_back(pid);
    i++;
  }

  RCLCPP_INFO(this->get_logger(), "Sending goal to FollowPath behavior");

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
      &PathPlanner::followPathResponseCbk, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&PathPlanner::followPathFeedbackCbk, this,
                std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
      std::bind(&PathPlanner::followPathResultCbk, this, std::placeholders::_1);
  follow_path_client_->async_send_goal(goal_msg, send_goal_options);
}

void PathPlanner::followPathResponseCbk(
    const GoalHandleFollowPath::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(
        this->get_logger(),
        "FollowPath was rejected by behavior server. Aborting navigation.");
    auto nav_result = std::make_shared<NavigateToPoint::Result>();
    nav_result->success = false;
    navigation_goal_handle_->abort(nav_result);
  } else {
    RCLCPP_INFO(this->get_logger(), "FollowPath accepted, flying to point.");
  }
}

void PathPlanner::followPathFeedbackCbk(
    GoalHandleFollowPath::SharedPtr goal_handle,
    const std::shared_ptr<const FollowPath::Feedback> feedback) {

  if (navigation_goal_handle_->is_canceling()) {
    // cancel follow path too
    follow_path_client_->async_cancel_goal(goal_handle);
    // propagate up the result of cancelling follow_path
    auto nav_result = std::make_shared<NavigateToPoint::Result>();
    nav_result->success = false;
    // TODO: not cancel navig inmediately, wait for follow_path cancel result
    navigation_goal_handle_->canceled(nav_result);
    return;
  }

  // TODO: current feedback is just a template
  auto nav_feedback = std::make_shared<NavigateToPoint::Feedback>();
  nav_feedback->current_pose = drone_pose_;
  nav_feedback->current_speed.twist.linear.x = feedback->actual_speed;
  nav_feedback->distance_remaining = feedback->actual_distance_to_next_waypoint;
  // nav_feedback->estimated_time_remaining = -1;
  // nav_feedback->navigation_time = -1;
  navigation_goal_handle_->publish_feedback(nav_feedback);
}

void PathPlanner::followPathResultCbk(
    const GoalHandleFollowPath::WrappedResult &result) {
  auto nav_result = std::make_shared<NavigateToPoint::Result>();
  nav_result->success = false;

  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(),
                 "FollowPath was aborted. Aborting navigation.");
    navigation_goal_handle_->abort(nav_result);
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(),
                 "FollowPath was canceled. Cancelling navigation");
    navigation_goal_handle_->canceled(nav_result);
    return;
  default:
    RCLCPP_ERROR(this->get_logger(),
                 "Unknown result code from FollowPath. Aborting navigation.");
    navigation_goal_handle_->abort(nav_result);
    return;
  }

  RCLCPP_INFO(
      this->get_logger(),
      "Follow Path succeeded. Goal point reached. Navigation succeeded.");
  nav_result->success = true;
  navigation_goal_handle_->succeed(nav_result);
}
