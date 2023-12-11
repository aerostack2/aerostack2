#include <drone_watcher.hpp>

DroneWatcher::DroneWatcher(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_ptr,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_ptr,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
        node_parameters_ptr,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_ptr,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_ptr,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_ptr,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ptr,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
        node_waitables_ptr,
    std::string drone_id)
    : node_base_ptr_(node_base_ptr), node_graph_ptr_(node_graph_ptr),
      node_parameters_ptr_(node_parameters_ptr),
      node_topics_ptr_(node_topics_ptr), node_services_ptr_(node_services_ptr),
      node_clock_ptr_(node_clock_ptr), node_logging_ptr_(node_logging_ptr),
      node_waitables_ptr_(node_waitables_ptr), drone_id_(drone_id) {

  rclcpp::SubscriptionOptions options;
  cbk_group_ = node_base_ptr->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = cbk_group_;

  drone_pose_sub_ =
      rclcpp::create_subscription<geometry_msgs::msg::PoseStamped>(
          node_parameters_ptr_, node_topics_ptr_,
          drone_id_ + "/self_localization/pose",
          as2_names::topics::self_localization::qos,
          std::bind(&DroneWatcher::dronePoseCbk, this, std::placeholders::_1),
          options);

  traj_gen_behavior_status_sub_ =
      rclcpp::create_subscription<as2_msgs::msg::BehaviorStatus>(
          node_parameters_ptr_, node_topics_ptr_,
          drone_id_ + "/TrajectoryGeneratorBehavior/_behavior/behavior_status",
          10,
          std::bind(&DroneWatcher::trajGenBehaviorStatusCbk, this,
                    std::placeholders::_1),
          options);

  follow_path_feedback_sub_ =
      rclcpp::create_subscription<as2_msgs::action::FollowPath_FeedbackMessage>(
          node_parameters_ptr_, node_topics_ptr_,
          drone_id_ + "/FollowPathBehavior/_action/feedback", 10,
          std::bind(&DroneWatcher::followPathFeedbackCbk, this,
                    std::placeholders::_1),
          options);

  pause_client_ = rclcpp::create_client<std_srvs::srv::Trigger>(
      node_base_ptr_, node_graph_ptr_, node_services_ptr_,
      drone_id_ + "/TrajectoryGeneratorBehavior/_behavior/pause",
      rmw_qos_profile_services_default, cbk_group_);

  resume_client_ = rclcpp::create_client<std_srvs::srv::Trigger>(
      node_base_ptr_, node_graph_ptr_, node_services_ptr_,
      drone_id_ + "/TrajectoryGeneratorBehavior/_behavior/resume",
      rmw_qos_profile_services_default, cbk_group_);

  std::string ns = drone_id_.erase(0, 1);
  go_to_client_ = rclcpp_action::create_client<GoTo>(
      node_base_ptr_, node_graph_ptr_, node_logging_ptr_, node_waitables_ptr_,
      ns + "/" + as2_names::actions::behaviors::gotowaypoint, cbk_group_);
}

void DroneWatcher::dronePoseCbk(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  drone_pose_ = *(msg);
}

void DroneWatcher::trajGenBehaviorStatusCbk(
    const as2_msgs::msg::BehaviorStatus::SharedPtr msg) {
  traj_gen_status = msg->status;
}

void DroneWatcher::followPathFeedbackCbk(
    const as2_msgs::action::FollowPath_FeedbackMessage::SharedPtr msg) {
  RCLCPP_INFO_ONCE(node_logging_ptr_->get_logger(),
                   "FollowPathFeedback received");
  follow_path_feedback_ = *(msg);
}

bool DroneWatcher::stop() {
  std_srvs::srv::Trigger::Request req;
  std_srvs::srv::Trigger::Response res;

  if (!pause_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_logging_ptr_->get_logger(), "Service %s not available.",
                 pause_client_->get_service_name());
    return false;
  }

  auto result = pause_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(req));
  result.wait(); // TODO: watch out, might block forever

  res = *(result.get());
  if (!res.success) {
    RCLCPP_ERROR(node_logging_ptr_->get_logger(), "Service %s failed.",
                 pause_client_->get_service_name());
    return false;
  }
  return true;
}

bool DroneWatcher::resume() {
  std_srvs::srv::Trigger::Request req;
  std_srvs::srv::Trigger::Response res;

  if (!resume_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_logging_ptr_->get_logger(), "Service %s not available.",
                 resume_client_->get_service_name());
    return false;
  }

  auto result = resume_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(req));
  result.wait(); // TODO: watch out, might block forever

  res = *(result.get());
  if (!res.success) {
    RCLCPP_ERROR(node_logging_ptr_->get_logger(), "Service %s failed.",
                 resume_client_->get_service_name());
    return false;
  }

  res.success = true;
  return true;
}

void DroneWatcher::moveVertically(double z_diff) {
  if (!go_to_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_logging_ptr_->get_logger(),
                 "GoTo Action server not available "
                 "after waiting. Aborting.");
    return;
  }

  auto goal_msg = GoTo::Goal();
  goal_msg.target_pose.header.frame_id = "earth";
  goal_msg.target_pose.header.stamp = node_clock_ptr_->get_clock()->now();
  goal_msg.target_pose.point.x = drone_pose_.pose.position.x;
  goal_msg.target_pose.point.y = drone_pose_.pose.position.y;
  goal_msg.target_pose.point.z = drone_pose_.pose.position.z + z_diff;
  goal_msg.yaw.mode = as2_msgs::msg::YawMode::KEEP_YAW;
  goal_msg.max_speed = 0.5;

  RCLCPP_INFO(node_logging_ptr_->get_logger(), "Sending goal to GoTo behavior");

  auto goal_handle_future = go_to_client_->async_send_goal(goal_msg);

  goal_handle_future.wait(); // TODO: might block forever
  auto goal_handle = goal_handle_future.get();
  if (goal_handle == nullptr) {
    RCLCPP_ERROR(node_logging_ptr_->get_logger(), "GoTo rejected from server");
    return;
  }

  // Waiting for go_to to end
  bool end = false;
  while (!end) {
    switch (goal_handle->get_status()) {
    case rclcpp_action::GoalStatus::STATUS_ABORTED:
      return;
      break;
    case rclcpp_action::GoalStatus::STATUS_CANCELED:
      return;
      break;
    case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
      end = true;
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
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(node_logging_ptr_->get_logger(), "GoTo ended");
  return;
}

void DroneWatcher::avoidanceManeuver() {
  stop();
  moveVertically(1.0);
}

void DroneWatcher::backToFollowPath() {
  moveVertically(-1.0);
  resume();
}