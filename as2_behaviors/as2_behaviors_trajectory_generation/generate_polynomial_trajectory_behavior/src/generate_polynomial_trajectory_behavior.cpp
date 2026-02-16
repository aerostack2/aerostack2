// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file generate_polynomial_trajectory_behavior.cpp
 *
 * @brief Source file for the GeneratePolynomialTrajectoryBehavior class.
 *
 * @author Miguel Fernández Cortizas
 *         Pedro Arias Pérez
 *         David Pérez Saura
 *         Rafael Pérez Seguí
 */

#include "generate_polynomial_trajectory_behavior.hpp"

DynamicPolynomialTrajectoryGenerator::DynamicPolynomialTrajectoryGenerator(
  const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<
    as2_msgs::action::GeneratePolynomialTrajectory>(
    as2_names::actions::behaviors::trajectorygenerator, options),
  trajectory_motion_handler_(this),
  hover_motion_handler_(this),
  tf_handler_(this)
{
  desired_frame_id_ = as2::tf::generateTfName(this, "odom");
  map_frame_id_ = as2::tf::generateTfName(this, "map");
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  trajectory_generator_ =
    std::make_shared<dynamic_traj_generator::DynamicTrajectory>();

  state_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::self_localization::twist,
    as2_names::topics::self_localization::qos,
    std::bind(
      &DynamicPolynomialTrajectoryGenerator::stateCallback, this,
      std::placeholders::_1));

  yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "traj_gen/yaw", rclcpp::SensorDataQoS(),
    std::bind(
      &DynamicPolynomialTrajectoryGenerator::yawCallback, this,
      std::placeholders::_1));

  /** For faster waypoint modified */
  mod_waypoint_sub_ =
    this->create_subscription<as2_msgs::msg::PoseStampedWithIDArray>(
    as2_names::topics::motion_reference::modify_waypoint,
    as2_names::topics::motion_reference::qos_waypoint,
    std::bind(
      &DynamicPolynomialTrajectoryGenerator::modifyWaypointCallback,
      this, std::placeholders::_1));

  // Read ROS 2 parameters
  this->declare_parameter<int>("sampling_n");
  sampling_n_ = this->get_parameter("sampling_n").as_int();
  this->declare_parameter<double>("sampling_dt");
  sampling_dt_ = this->get_parameter("sampling_dt").as_double();
  path_length_ = this->declare_parameter<int>("path_length", 0);
  yaw_threshold_ = this->declare_parameter<float>("yaw_threshold", 0.0);
  transform_threshold_ = this->declare_parameter<float>("transform_threshold", 1.0);
  yaw_speed_threshold_ = this->declare_parameter<double>("yaw_speed_threshold", 2.0);
  frequency_update_frame_ = this->declare_parameter<double>("frequency_update_frame", 0.0);
  wp_close_threshold_ = this->declare_parameter<double>("wp_close_threshold", 0.0);

  if (sampling_n_ < 1) {
    RCLCPP_ERROR(this->get_logger(), "Sampling n must be greater than 0");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Sampling with n = %d and dt = %f", sampling_n_, sampling_dt_);

  /** Debug publishers **/
  std::string path_topic = this->declare_parameter<std::string>("debug.path_topic", "");
  std::string reference_setpoint = this->declare_parameter<std::string>(
    "debug.reference_setpoint",
    "");
  std::string reference_end_waypoint = this->declare_parameter<std::string>(
    "debug.reference_end_waypoint", "");
  std::string reference_waypoints = this->declare_parameter<std::string>(
    "debug.reference_waypoints", "");

  if (path_topic != "") {
    debug_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 1);
  }

  if (reference_setpoint != "") {
    debug_ref_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      reference_setpoint, 1);
  }

  if (reference_end_waypoint != "") {
    debug_end_ref_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      reference_end_waypoint, 1);
  }

  if (reference_waypoints != "") {
    debug_waypoints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      reference_waypoints, 1);
  }

  if (debug_path_pub_ != nullptr || debug_ref_point_pub_ != nullptr ||
    debug_end_ref_point_pub_ != nullptr || debug_waypoints_pub_ != nullptr)
  {
    enable_debug_ = true;
  }
  return;
}
void DynamicPolynomialTrajectoryGenerator::timerUpdateFrameCallback()
{
  if (computeErrorFrames()) {
    if (!updateFrame(goal_)) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not update transform between %s and %s",
        map_frame_id_.c_str(), desired_frame_id_.c_str());
    }
  }
}
void DynamicPolynomialTrajectoryGenerator::stateCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg)
{
  try {
    geometry_msgs::msg::PoseStamped pose_msg =
      tf_handler_.getPoseStamped(
      desired_frame_id_, base_link_frame_id_,
      tf2_ros::fromMsg(_twist_msg->header.stamp));

    if (!has_odom_) {
      RCLCPP_INFO(this->get_logger(), "State callback working");
      has_odom_ = true;
    }

    current_position_ =
      Eigen::Vector3d(
      pose_msg.pose.position.x, pose_msg.pose.position.y,
      pose_msg.pose.position.z);
    current_yaw_ = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);
    trajectory_generator_->updateVehiclePosition(
      Eigen::Vector3d(
        pose_msg.pose.position.x, pose_msg.pose.position.y,
        pose_msg.pose.position.z));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

void DynamicPolynomialTrajectoryGenerator::yawCallback(
  const std_msgs::msg::Float32::SharedPtr _msg)
{
  has_yaw_from_topic_ = true;
  yaw_from_topic_ = _msg->data;
}

bool DynamicPolynomialTrajectoryGenerator::goalToDynamicWaypoint(
  std::shared_ptr<const as2_msgs::action::GeneratePolynomialTrajectory::Goal>
  _goal,
  dynamic_traj_generator::DynamicWaypoint::Deque & _waypoints_to_set)
{
  std::vector<std::string> waypoint_ids;
  waypoint_ids.reserve(_goal->path.size());

  if (_goal->max_speed < 0) {
    RCLCPP_ERROR(this->get_logger(), "Goal max speed is negative");
    return false;
  }
  trajectory_generator_->setSpeed(_goal->max_speed);
  _waypoints_to_set.clear();
  for (as2_msgs::msg::PoseStampedWithID waypoint : _goal->path) {
    // Process each waypoint id
    if (waypoint.id == "") {
      RCLCPP_ERROR(this->get_logger(), "Waypoint ID is empty");
      return false;
    } else {
      // Else if waypoint ID is in the list of waypoint IDs, then return false
      if (std::find(
          waypoint_ids.begin(), waypoint_ids.end(),
          waypoint.id) != waypoint_ids.end())
      {
        RCLCPP_ERROR(
          this->get_logger(), "Waypoint ID %s is not unique",
          waypoint.id.c_str());
        return false;
      }
    }

    waypoint_ids.push_back(waypoint.id);

    // Process each waypoint frame_id
    if (waypoint.pose.header.frame_id != desired_frame_id_) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      try {
        waypoint.pose = tf_handler_.convert(waypoint.pose, desired_frame_id_);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(
          this->get_logger(), "Could not get transform: %s",
          ex.what());
        return false;
      }
    }

    // Set to dynamic trajectory generator
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    generateDynamicPoint(waypoint, dynamic_waypoint);
    _waypoints_to_set.emplace_back(dynamic_waypoint);
  }
  return true;
}

bool DynamicPolynomialTrajectoryGenerator::on_activate(
  std::shared_ptr<const as2_msgs::action::GeneratePolynomialTrajectory::Goal>
  goal)
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator on activate");

  if (!has_odom_) {
    RCLCPP_ERROR(this->get_logger(), "No odometry information available");
    return false;
  }
  if (frequency_update_frame_ > 0) {
    /** Calback to check the transform between frames */
    timer_update_frame_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000 / frequency_update_frame_)),
      std::bind(
        &DynamicPolynomialTrajectoryGenerator::timerUpdateFrameCallback,
        this));
    // wait until needed
    timer_update_frame_->cancel();
  }
  setup();

  // Print goal path
  for (auto waypoint : goal->path) {
    RCLCPP_INFO(
      this->get_logger(),
      "Waypoint ID: %s, frame_id: %s, position : [%f, %f, %f]",
      waypoint.id.c_str(), waypoint.pose.header.frame_id.c_str(),
      waypoint.pose.pose.position.x, waypoint.pose.pose.position.y,
      waypoint.pose.pose.position.z);
  }

  if (!goalToDynamicWaypoint(goal, waypoints_to_set_)) {return false;}
  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  if (path_length_ == 0 || (goal->path.size() < (path_length_ + 1))) {
    waypoints_to_set.reserve(waypoints_to_set_.size());
    for (dynamic_traj_generator::DynamicWaypoint wp : waypoints_to_set_) {
      waypoints_to_set.emplace_back(wp);
    }
    waypoints_to_set_.clear();
  } else {
    waypoints_to_set.reserve(path_length_ + 1);
    for (int i = 0; i < path_length_ + 1; i++) {
      waypoints_to_set.emplace_back(waypoints_to_set_.front());
      waypoints_to_set_.pop_front();
    }
  }
  // Set waypoints to trajectory generator
  trajectory_generator_->setWaypoints(waypoints_to_set);
  last_map_to_odom_transform_ = tf_handler_.getTransform(
    map_frame_id_, desired_frame_id_,
    tf2::TimePointZero);
  yaw_mode_ = goal->yaw;
  goal_ = *goal;

  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator goal accepted");
  return true;
}

void DynamicPolynomialTrajectoryGenerator::setup()
{
  // trajectory_generator_ =
  //     std::make_shared<dynamic_traj_generator::DynamicTrajectory>();
  has_yaw_from_topic_ = false;
  has_odom_ = false;
  first_run_ = true;

  trajectory_command_ = as2_msgs::msg::TrajectorySetpoints();
  trajectory_command_.header.frame_id = desired_frame_id_;
  trajectory_command_.setpoints.resize(sampling_n_);
  time_zero_yaw_ = this->now();
  init_yaw_angle_ = current_yaw_;
  if (frequency_update_frame_ > 0) {
    timer_update_frame_->reset();
  }
}

bool DynamicPolynomialTrajectoryGenerator::on_modify(
  std::shared_ptr<const as2_msgs::action::GeneratePolynomialTrajectory::Goal>
  goal)
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator goal modified");

  // Generate queue of waypoints for trajectory generator, from goal to
  // dynamic_traj_generator::DynamicWaypoint::Deque
  waypoints_to_set_.clear();
  if (!goalToDynamicWaypoint(goal, waypoints_to_set_)) {return false;}
  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  if (path_length_ == 0 || (goal->path.size() < (path_length_ + 1))) {
    waypoints_to_set.reserve(waypoints_to_set_.size());
    for (dynamic_traj_generator::DynamicWaypoint wp : waypoints_to_set_) {
      waypoints_to_set.emplace_back(wp);
    }
    waypoints_to_set_.clear();
  } else {
    waypoints_to_set.reserve(path_length_ + 1);
    for (int i = 0; i < path_length_ + 1; i++) {
      waypoints_to_set.emplace_back(waypoints_to_set_.front());
      waypoints_to_set_.pop_front();
    }
  }


  // Modify each waypoint
  for (dynamic_traj_generator::DynamicWaypoint dynamic_waypoint :
    waypoints_to_set)
  {
    trajectory_generator_->modifyWaypoint(
      dynamic_waypoint.getName(), dynamic_waypoint.getCurrentPosition());

    // DEBUG
    RCLCPP_INFO(
      this->get_logger(), "waypoint[%s] added: (%.2f, %.2f, %.2f)",
      dynamic_waypoint.getName().c_str(),
      dynamic_waypoint.getOriginalPosition().x(),
      dynamic_waypoint.getOriginalPosition().y(),
      dynamic_waypoint.getOriginalPosition().z());
  }

  return true;
}

/** For faster waypoint modified */
void DynamicPolynomialTrajectoryGenerator::modifyWaypointCallback(
  const as2_msgs::msg::PoseStampedWithIDArray::SharedPtr _msg)
{
  for (as2_msgs::msg::PoseStampedWithID waypoint : _msg->poses) {
    geometry_msgs::msg::PoseStamped pose_stamped = waypoint.pose;

    if (pose_stamped.header.frame_id != desired_frame_id_) {
      try {
        pose_stamped = tf_handler_.convert(pose_stamped, desired_frame_id_);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return;
      }
    }

    Eigen::Vector3d position;
    position.x() = pose_stamped.pose.position.x;
    position.y() = pose_stamped.pose.position.y;
    position.z() = pose_stamped.pose.position.z;
    trajectory_generator_->modifyWaypoint(waypoint.id, position);
    RCLCPP_DEBUG(
      this->get_logger(), "waypoint[%s] modified: %s - (%.2f, %.2f, %.2f)",
      waypoint.id.c_str(), pose_stamped.header.frame_id.c_str(),
      position.x(), position.y(), position.z());
  }
}

bool DynamicPolynomialTrajectoryGenerator::on_deactivate(
  const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator cancelled");
  return true;
}

bool DynamicPolynomialTrajectoryGenerator::on_pause(
  const std::shared_ptr<std::string> & message)
{
  RCLCPP_WARN(
    this->get_logger(),
    "TrajectoryGenerator can not be paused, try "
    "to cancel it and start a new one");
  if (frequency_update_frame_ > 0) {
    timer_update_frame_->cancel();
  }
  // Reset the trajectory generator
  trajectory_generator_ =
    std::make_shared<dynamic_traj_generator::DynamicTrajectory>();

  // Send the hover motion command
  hover_motion_handler_.sendHover();

  return true;
}

bool DynamicPolynomialTrajectoryGenerator::on_resume(
  const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator resume from pause");

  if (!has_odom_) {
    RCLCPP_ERROR(this->get_logger(), "No odometry information available");
    return false;
  }
  setup();

  // Print goal path
  bool start_trajectory = false;
  auto paused_goal = as2_msgs::action::GeneratePolynomialTrajectory::Goal();
  paused_goal.stamp = this->now();
  paused_goal.yaw = goal_.yaw;
  paused_goal.max_speed = goal_.max_speed;

  paused_goal.path = std::vector<as2_msgs::msg::PoseStampedWithID>();
  for (auto waypoint : goal_.path) {
    if (waypoint.id != feedback_.next_waypoint_id && !start_trajectory) {
      continue;
    }
    start_trajectory = true;
    paused_goal.path.push_back(waypoint);

    RCLCPP_INFO(
      this->get_logger(),
      "Waypoint remainings ID: %s, frame_id: %s, position : [%f, %f, %f]",
      waypoint.id.c_str(), waypoint.pose.header.frame_id.c_str(),
      waypoint.pose.pose.position.x, waypoint.pose.pose.position.y,
      waypoint.pose.pose.position.z);
  }

  if (paused_goal.path.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "No waypoint remaining");
    return false;
  }

  // Generate queue of waypoints for trajectory generator, from goal to
  // dynamic_traj_generator::DynamicWaypoint::Deque
  auto paused_goal_shared_ptr =
    std::make_shared<as2_msgs::action::GeneratePolynomialTrajectory::Goal>(
    paused_goal);
  waypoints_to_set_.clear();
  if (!goalToDynamicWaypoint(paused_goal_shared_ptr, waypoints_to_set_)) {
    return false;
  }
  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  if (path_length_ == 0 || (paused_goal.path.size() < (path_length_ + 1))) {
    waypoints_to_set.reserve(waypoints_to_set_.size());
    for (dynamic_traj_generator::DynamicWaypoint wp : waypoints_to_set_) {
      waypoints_to_set.emplace_back(wp);
    }
    waypoints_to_set_.clear();
  } else {
    waypoints_to_set.reserve(path_length_ + 1);
    for (int i = 0; i < path_length_ + 1; i++) {
      waypoints_to_set.emplace_back(waypoints_to_set_.front());
      waypoints_to_set_.pop_front();
    }
  }

  // Set waypoints to trajectory generator
  trajectory_generator_->setWaypoints(waypoints_to_set);
  yaw_mode_ = paused_goal.yaw;
  goal_ = paused_goal;

  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator resumed");
  return true;
}

void DynamicPolynomialTrajectoryGenerator::on_execution_end(
  const as2_behavior::ExecutionStatus & state)
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator end");
  if (frequency_update_frame_ > 0) {
    timer_update_frame_.reset();
  }
  // Reset the trajectory generator
  trajectory_generator_ =
    std::make_shared<dynamic_traj_generator::DynamicTrajectory>();
  // Reset the queue of waypoints
  waypoints_to_set_.clear();
  if (state == as2_behavior::ExecutionStatus::SUCCESS ||
    state == as2_behavior::ExecutionStatus::ABORTED)
  {
    return;
  }

  // If the trajectory generator is not successful, we send a hover command
  hover_motion_handler_.sendHover();
  return;
}

as2_behavior::ExecutionStatus DynamicPolynomialTrajectoryGenerator::on_run(
  const std::shared_ptr<
    const as2_msgs::action::GeneratePolynomialTrajectory::Goal> & goal,
  std::shared_ptr<as2_msgs::action::GeneratePolynomialTrajectory::Feedback>
  & feedback_msg,
  std::shared_ptr<as2_msgs::action::GeneratePolynomialTrajectory::Result>
  & result_msg)
{
  bool publish_trajectory = false;
  if (first_run_) {
    publish_trajectory = evaluateTrajectory(trajectory_generator_->getMinTime());
    time_zero_ = this->now();
    eval_time_ = rclcpp::Duration(0, 0);
    first_run_ = false;
  } else {
    if (!trajectory_generator_->getGenerateNewTraj()) {
      if (path_length_ > 0 && goal->path.size() > this->path_length_) {
        if (!waypoints_to_set_.empty()) {
          if (trajectory_generator_->getRemainingWaypoints() < path_length_) {
            trajectory_generator_->appendWaypoint(waypoints_to_set_.front());
            waypoints_to_set_.pop_front();
          }
        }
      }
    }
    eval_time_ = this->now() - time_zero_;
    publish_trajectory = evaluateTrajectory(eval_time_.seconds());
  }

  // Check success trajectory generator evaluation
  if (!publish_trajectory) {
    // TODO(CVAR): When trajectory_generator_->evaluateTrajectory == False?
    result_msg->trajectory_generator_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  // Check if the trajectory generator has finished
  if (trajectory_generator_->getMaxTime() < (eval_time_.seconds() - 0.01) &&
    !first_run_ && !trajectory_generator_->getGenerateNewTraj())
  {
    if (waypoints_to_set_.empty()) {
      result_msg->trajectory_generator_success = true;
      return as2_behavior::ExecutionStatus::SUCCESS;
    } else {
      trajectory_generator_->appendWaypoint(waypoints_to_set_.front());
      waypoints_to_set_.pop_front();
      eval_time_ = this->now() - time_zero_;
      publish_trajectory = evaluateTrajectory(eval_time_.seconds());
      RCLCPP_ERROR(this->get_logger(), "Trajectory has not generated all the waypoints ");
      // TO DO: carmendrpr
    }
  }

  // Plot debug trajectory
  if (enable_debug_) {
    plotRefTrajPoint();
    if ((debug_path_pub_ != nullptr || debug_waypoints_pub_ != nullptr) &&
      trajectory_generator_->getWasTrajectoryRegenerated())
    {
      plotTrajectory();
    }
  }

  // Publish trajectory motion reference
  if (!trajectory_motion_handler_.sendTrajectorySetpoints(trajectory_command_)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "TrajectoryGenerator: Could not send trajectory command");
    result_msg->trajectory_generator_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  auto next_trajectory_waypoints =
    trajectory_generator_->getNextTrajectoryWaypoints();

  feedback_.remaining_waypoints = next_trajectory_waypoints.size();
  if (feedback_.remaining_waypoints > 0) {
    feedback_.next_waypoint_id = next_trajectory_waypoints[0].getName();
  } else {
    feedback_.next_waypoint_id = "";
  }

  feedback_msg->remaining_waypoints = feedback_.remaining_waypoints;
  feedback_msg->next_waypoint_id = feedback_.next_waypoint_id;
  return as2_behavior::ExecutionStatus::RUNNING;
}

bool DynamicPolynomialTrajectoryGenerator::evaluateTrajectory(
  double eval_time)
{
  as2_msgs::msg::TrajectoryPoint setpoint;
  for (int i = 0; i < sampling_n_; i++) {
    bool success;
    if (i == 0) {
      success = evaluateSetpoint(eval_time, setpoint);
    } else {
      success = evaluateSetpoint(eval_time, setpoint, false);
    }
    if (!success) {
      return false;
    }
    trajectory_command_.setpoints[i] = setpoint;
    eval_time += sampling_dt_;
  }
  trajectory_command_.header.stamp = this->now();
  return true;
}

bool DynamicPolynomialTrajectoryGenerator::evaluateSetpoint(
  double eval_time,
  as2_msgs::msg::TrajectoryPoint & setpoint,
  bool current_setpoint)
{
  dynamic_traj_generator::References traj_command;
  double yaw_angle;

  if (eval_time <= trajectory_generator_->getMinTime()) {
    eval_time = trajectory_generator_->getMinTime();
  } else if (eval_time >= trajectory_generator_->getMaxTime()) {
    eval_time = trajectory_generator_->getMaxTime();
  }
  bool succes_eval =
    trajectory_generator_->evaluateTrajectory(eval_time, traj_command, false, !current_setpoint);

  switch (yaw_mode_.mode) {
    case as2_msgs::msg::YawMode::FACE_REFERENCE:
      yaw_angle = computeYawFaceReference();
      break;
    case as2_msgs::msg::YawMode::KEEP_YAW:
      yaw_angle = init_yaw_angle_;
      break;
    case as2_msgs::msg::YawMode::PATH_FACING:
      yaw_angle = computeYawAnglePathFacing(
        traj_command.velocity.x(),
        traj_command.velocity.y());
      break;
    case as2_msgs::msg::YawMode::FIXED_YAW:
      yaw_angle = yaw_mode_.angle;
      break;
    case as2_msgs::msg::YawMode::YAW_FROM_TOPIC:
      if (has_yaw_from_topic_) {
        yaw_angle = yaw_from_topic_;
      } else {
        auto & clk = *this->get_clock();
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), clk, 1000,
          "Yaw from topic not received yet, using last yaw angle");
        yaw_angle = init_yaw_angle_;
      }
      break;
    default:
      auto & clk = *this->get_clock();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), clk, 5000,
        "Unknown yaw mode, using keep yaw");
      yaw_angle = current_yaw_;
      break;
  }

  setpoint.position.x = traj_command.position.x();
  setpoint.position.y = traj_command.position.y();
  setpoint.position.z = traj_command.position.z();
  setpoint.twist.x = traj_command.velocity.x();
  setpoint.twist.y = traj_command.velocity.y();
  setpoint.twist.z = traj_command.velocity.z();
  setpoint.acceleration.x = traj_command.acceleration.x();
  setpoint.acceleration.y = traj_command.acceleration.y();
  setpoint.acceleration.z = traj_command.acceleration.z();
  setpoint.yaw_angle = yaw_angle;

  return succes_eval;
}
bool DynamicPolynomialTrajectoryGenerator::updateFrame(
  const as2_msgs::action::GeneratePolynomialTrajectory::Goal &
  goal)
{
  std::vector<std::pair<std::string, Eigen::Vector3d>> waypoints_to_set_vector;
  // Update the frame
  for (as2_msgs::msg::PoseStampedWithID waypoint : goal.path) {
    if (!tf_handler_.tryConvert(waypoint.pose, desired_frame_id_)) {return false;}
    // Update the waypoint in the trajectory generator
    for (auto traj_waypoint : trajectory_generator_->getNextTrajectoryWaypoints()) {
      if (traj_waypoint.getName() == waypoint.id) {
        if ((traj_waypoint.getTime() - eval_time_.seconds()) > wp_close_threshold_) {
          waypoint.pose.header.stamp = this->now();
          Eigen::Vector3d position;
          position.x() = waypoint.pose.pose.position.x;
          position.y() = waypoint.pose.pose.position.y;
          position.z() = waypoint.pose.pose.position.z;
          waypoints_to_set_vector.emplace_back(waypoint.id, position);
        }
      }
    }
    // Update the waypoints in the queue
    dynamic_traj_generator::DynamicWaypoint::Deque waypoints_to_set_queue;
    for (auto waypoints_queue : waypoints_to_set_) {
      if (waypoints_queue.getName() == waypoint.id) {
        waypoints_to_set_queue.emplace_back(waypoints_queue);
      }
    }
    waypoints_to_set_.clear();
    waypoints_to_set_ = waypoints_to_set_queue;
  }
  last_map_to_odom_transform_ = tf_handler_.getTransform(
    map_frame_id_, desired_frame_id_,
    tf2::TimePointZero);
  trajectory_generator_->modifyWaypoints(waypoints_to_set_vector);
  time_debug_ = this->now();
  waypoints_to_set_vector.clear();
  return true;
}

double DynamicPolynomialTrajectoryGenerator::computeYawAnglePathFacing(
  double vx, double vy)
{
  if (sqrt(vx * vx + vy * vy) > yaw_threshold_) {
    return as2::frame::getVector2DAngle(vx, vy);
  }
  return current_yaw_;
}
double DynamicPolynomialTrajectoryGenerator::computeYawFaceReference()
{
  eval_time_yaw_ = rclcpp::Duration(0, 0);
  if (trajectory_generator_->getRemainingWaypoints() < 1) {
    time_zero_yaw_ = this->now();
    return current_yaw_;
  }

  dynamic_traj_generator::DynamicWaypoint::Vector next_trajectory_waypoint =
    trajectory_generator_->getNextTrajectoryWaypoints();
  Eigen::Vector3d next_position = next_trajectory_waypoint.begin()->getOriginalPosition();
  Eigen::Vector2d diff(next_position[0] - current_position_[0],
    next_position[1] - current_position_[1]);

  if (diff.norm() > yaw_threshold_) {
    // Error from the current yaw to the desired yaw
    double yaw_error =
      as2::frame::angleMinError(
      as2::frame::getVector2DAngle(diff.x(), diff.y()), current_yaw_);
    eval_time_yaw_ = this->now() - time_zero_yaw_;
    // Compute the speed to reach the desired yaw
    double yaw_speed = yaw_error / eval_time_yaw_.seconds();
    // Limit the yaw speed
    yaw_speed = std::clamp(yaw_speed, (-yaw_speed_threshold_), yaw_speed_threshold_);
    // Store the time when the yaw is set
    time_zero_yaw_ = this->now();
    return current_yaw_ + yaw_speed * eval_time_yaw_.seconds();
  } else {
    // Store the time when the yaw is set
    time_zero_yaw_ = this->now();
    // If there are no more waypoints, keep the current yaw
    return current_yaw_;
  }
}
bool DynamicPolynomialTrajectoryGenerator::computeErrorFrames()
{
  current_map_to_odom_transform_ =
    tf_handler_.getTransform(map_frame_id_, desired_frame_id_, tf2::TimePointZero);
  Eigen::Vector3d current_traslation = Eigen::Vector3d(
    current_map_to_odom_transform_.transform.translation.x,
    current_map_to_odom_transform_.transform.translation.y,
    current_map_to_odom_transform_.transform.translation.z);
  Eigen::Vector3d last_traslation = Eigen::Vector3d(
    last_map_to_odom_transform_.transform.translation.x,
    last_map_to_odom_transform_.transform.translation.y,
    last_map_to_odom_transform_.transform.translation.z);
  double current_yaw = as2::frame::getYawFromQuaternion(
    current_map_to_odom_transform_.transform.rotation);
  double last_yaw =
    as2::frame::getYawFromQuaternion(last_map_to_odom_transform_.transform.rotation);
  Eigen::Vector3d traslation_error =
    Eigen::Vector3d(
    current_traslation[0] - last_traslation[0],
    current_traslation[1] - last_traslation[1],
    current_traslation[2] - last_traslation[2]);
  if (traslation_error.norm() > transform_threshold_) {
    return true;
  }
  // if (as2::frame::angleMinError(current_yaw, last_yaw) > 0.5) {
  //   return true;
  // }
  return false;
}

/** Debug functions **/

void DynamicPolynomialTrajectoryGenerator::plotTrajectory()
{
  // launch async plot
  if (plot_thread_.joinable()) {
    plot_thread_.join();
  }
  plot_thread_ = std::thread(
    &DynamicPolynomialTrajectoryGenerator::plotTrajectoryThread, this);
}

void DynamicPolynomialTrajectoryGenerator::plotTrajectoryThread()
{
  rclcpp::Time time_stamp;
  if (time_debug_.has_value()) {
    time_stamp = time_debug_.value();
  } else {
    time_stamp = this->now();
  }

  if (debug_path_pub_ != nullptr) {
    nav_msgs::msg::Path path_msg;
    const float step = 0.2;
    const float max_time = trajectory_generator_->getMaxTime();
    const float min_time = trajectory_generator_->getMinTime();
    dynamic_traj_generator::References refs;
    const int n_measures = (max_time - min_time) / step;
    // path_msg.poses.reserve(n_measures);
    for (float time = min_time; time <= max_time; time += step) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.frame_id = desired_frame_id_;
      pose_msg.header.stamp = time_stamp;
      trajectory_generator_->evaluateTrajectory(time, refs, true, true);
      pose_msg.pose.position.x = refs.position.x();
      pose_msg.pose.position.y = refs.position.y();
      pose_msg.pose.position.z = refs.position.z();
      path_msg.poses.emplace_back(pose_msg);
    }
    path_msg.header.frame_id = desired_frame_id_;
    path_msg.header.stamp = time_stamp;

    RCLCPP_INFO(this->get_logger(), "DEBUG: Plotting trajectory");
    debug_path_pub_->publish(path_msg);
  }

  // Plot waypoints
  if (debug_waypoints_pub_ != nullptr) {
    visualization_msgs::msg::MarkerArray waypoints_msg;
    visualization_msgs::msg::Marker waypoint_msg;
    waypoint_msg.header.frame_id = desired_frame_id_;
    waypoint_msg.header.stamp = time_stamp;
    waypoint_msg.type = visualization_msgs::msg::Marker::SPHERE;
    waypoint_msg.action = visualization_msgs::msg::Marker::ADD;
    waypoint_msg.color.r = 1.0f;
    waypoint_msg.color.g = 0.0f;
    waypoint_msg.color.b = 0.0f;
    waypoint_msg.color.a = 1.0f;
    waypoint_msg.scale.x = 0.1;
    waypoint_msg.scale.y = 0.1;
    waypoint_msg.scale.z = 0.1;

    dynamic_traj_generator::DynamicWaypoint::Deque waypoints_to_set;
    std::shared_ptr<const as2_msgs::action::GeneratePolynomialTrajectory::Goal> goal;
    goal = std::make_shared<const as2_msgs::action::GeneratePolynomialTrajectory::Goal>(goal_);
    if (!goalToDynamicWaypoint(goal, waypoints_to_set)) {return;}

    int id = 0;
    for (dynamic_traj_generator::DynamicWaypoint wp : waypoints_to_set) {
      waypoint_msg.pose.position.x = wp.getCurrentPosition().x();
      waypoint_msg.pose.position.y = wp.getCurrentPosition().y();
      waypoint_msg.pose.position.z = wp.getCurrentPosition().z();
      waypoint_msg.id = id++;
      waypoints_msg.markers.emplace_back(waypoint_msg);
      RCLCPP_DEBUG(
        this->get_logger(),
        "Waypoint ID: %s, position : [%f, %f, %f]", wp.getName().c_str(),
        waypoint_msg.pose.position.x, waypoint_msg.pose.position.y,
        waypoint_msg.pose.position.z);
    }
    debug_waypoints_pub_->publish(waypoints_msg);
  }
}

void DynamicPolynomialTrajectoryGenerator::plotRefTrajPoint()
{
  visualization_msgs::msg::Marker point_msg;

  point_msg.header.frame_id = desired_frame_id_;
  point_msg.header.stamp = this->now();
  point_msg.type = visualization_msgs::msg::Marker::SPHERE;
  point_msg.action = visualization_msgs::msg::Marker::ADD;

  // Blue
  point_msg.color.r = 0.0f;
  point_msg.color.g = 0.0f;
  point_msg.color.b = 1.0f;
  point_msg.color.a = 1.0f;

  point_msg.scale.x = 0.1;
  point_msg.scale.y = 0.1;
  point_msg.scale.z = 0.1;

  if (debug_ref_point_pub_ != nullptr) {
    point_msg.pose.position.x = trajectory_command_.setpoints[0].position.x;
    point_msg.pose.position.y = trajectory_command_.setpoints[0].position.y;
    point_msg.pose.position.z = trajectory_command_.setpoints[0].position.z;

    debug_ref_point_pub_->publish(point_msg);
  }

  if (debug_end_ref_point_pub_ != nullptr) {
    // If more than one setpoint, plot the last one
    if (trajectory_command_.setpoints.size() < 2) {
      return;
    }
    // Green
    point_msg.color.g = 1.0f;
    point_msg.color.b = 0.0f;
    point_msg.pose.position.x =
      trajectory_command_.setpoints[trajectory_command_.setpoints.size() - 1].position.x;
    point_msg.pose.position.y =
      trajectory_command_.setpoints[trajectory_command_.setpoints.size() - 1].position.y;
    point_msg.pose.position.z =
      trajectory_command_.setpoints[trajectory_command_.setpoints.size() - 1].position.z;
    debug_end_ref_point_pub_->publish(point_msg);
  }
}

/** Auxiliar Functions **/
void generateDynamicPoint(
  const as2_msgs::msg::PoseStampedWithID & msg,
  dynamic_traj_generator::DynamicWaypoint & dynamic_point)
{
  dynamic_point.setName(msg.id);
  Eigen::Vector3d position;
  position.x() = msg.pose.pose.position.x;
  position.y() = msg.pose.pose.position.y;
  position.z() = msg.pose.pose.position.z;
  dynamic_point.resetWaypoint(position);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(DynamicPolynomialTrajectoryGenerator)
