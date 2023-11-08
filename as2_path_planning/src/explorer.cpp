#include <explorer.hpp>

Explorer::Explorer() : Node("explorer") {
  this->declare_parameter("frontier_min_area", 1);
  frontier_min_area_ = this->get_parameter("frontier_min_area").as_int();

  this->declare_parameter("safety_distance", 1.0); // aprox drone size [m]
  safety_distance_ = this->get_parameter("safety_distance").as_double();

  this->declare_parameter("reached_dist_thresh", 0.5);
  reached_dist_thresh_ = this->get_parameter("reached_dist_thresh").as_double();

  this->declare_parameter("navigation_speed", 1.0);
  navigation_speed_ = this->get_parameter("navigation_speed").as_double();

  occ_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map_server/map", 1,
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

  start_explore_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "start_exploration",
      std::bind(&Explorer::startExplorationCbk, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cbk_group_);

  ask_frontier_cli_ = this->create_client<as2_msgs::srv::AllocateFrontier>(
      "/allocate_frontier");

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
  while (utils::distance(drone_pose_.pose.position, point->point) >
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

/*
 * Navigate to the closest reachable frontier centroid to the given point.
 * Returns:
 *    0: navigation ended successfully
 *    1: navigation failed
 *   -1: navigation rejected by server
 *   -2: no frontiers found
 */
int Explorer::explore(geometry_msgs::msg::PointStamped goal) {
  // TODO: what if the request is rejected?
  int result = navigateTo(getFrontier(goal));
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
  goal_msg.navigation_speed = navigation_speed_;
  goal_msg.yaw.mode = as2_msgs::msg::YawMode::PATH_FACING;
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

geometry_msgs::msg::PointStamped
Explorer::getFrontier(const geometry_msgs::msg::PoseStamped &goal) {
  as2_msgs::srv::AllocateFrontier::Request::SharedPtr request =
      std::make_shared<as2_msgs::srv::AllocateFrontier::Request>();
  request->explorer_pose = goal;
  request->explorer_id = this->get_namespace();

  auto result = ask_frontier_cli_->async_send_request(request);
  result.wait();
  return result.get()->frontier;
}

geometry_msgs::msg::PointStamped
Explorer::getFrontier(const geometry_msgs::msg::PointStamped &goal) {
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = goal.header;
  goal_pose.pose.position = goal.point;
  return getFrontier(goal_pose);
}
