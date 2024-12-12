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
    this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
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

  if (sampling_n_ < 1) {
    RCLCPP_ERROR(this->get_logger(), "Sampling n must be greater than 0");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Sampling with n = %d and dt = %f", sampling_n_, sampling_dt_);

  /** Debug publishers **/
  ref_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(
    REF_TRAJ_TOPIC, 1);

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(PATH_DEBUG_TOPIC, 1);
  return;
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
  dynamic_traj_generator::DynamicWaypoint::Vector & _waypoints_to_set)
{
  std::vector<std::string> waypoint_ids;
  waypoint_ids.reserve(_goal->path.size());

  if (_goal->max_speed < 0) {
    RCLCPP_ERROR(this->get_logger(), "Goal max speed is negative");
    return false;
  }
  trajectory_generator_->setSpeed(_goal->max_speed);

  for (as2_msgs::msg::PoseWithID waypoint : _goal->path) {
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
    if (_goal->header.frame_id != desired_frame_id_) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = _goal->header;
      pose_stamped.pose = waypoint.pose;

      try {
        pose_stamped = tf_handler_.convert(pose_stamped, desired_frame_id_);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(
          this->get_logger(), "Could not get transform: %s",
          ex.what());
        return false;
      }

      waypoint.pose = pose_stamped.pose;
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
  setup();

  // Print goal path
  for (auto waypoint : goal->path) {
    RCLCPP_INFO(
      this->get_logger(), "Waypoint ID: %s, position : [%f, %f, %f]",
      waypoint.id.c_str(), waypoint.pose.position.x,
      waypoint.pose.position.y, waypoint.pose.position.z);
  }

  // Generate vector of waypoints for trajectory generator, from goal to
  // dynamic_traj_generator::DynamicWaypoint::Vector
  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  waypoints_to_set.reserve(goal->path.size() + 1);

  // // First waypoint is current position
  // dynamic_traj_generator::DynamicWaypoint initial_wp;
  // initial_wp.resetWaypoint(current_position_);
  // initial_wp.setName("initial_position");
  // waypoints_to_set.emplace_back(initial_wp);

  if (!goalToDynamicWaypoint(goal, waypoints_to_set)) {return false;}

  // Set waypoints to trajectory generator
  trajectory_generator_->setWaypoints(waypoints_to_set);

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
  init_yaw_angle_ = current_yaw_;
}

bool DynamicPolynomialTrajectoryGenerator::on_modify(
  std::shared_ptr<const as2_msgs::action::GeneratePolynomialTrajectory::Goal>
  goal)
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator goal modified");

  // Generate vector of waypoints for trajectory generator, from goal to
  // dynamic_traj_generator::DynamicWaypoint::Vector
  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  waypoints_to_set.reserve(goal->path.size());

  if (!goalToDynamicWaypoint(goal, waypoints_to_set)) {return false;}

  // Modify each waypoint
  for (dynamic_traj_generator::DynamicWaypoint dynamic_waypoint :
    waypoints_to_set)
  {
    trajectory_generator_->modifyWaypoint(
      dynamic_waypoint.getName(), dynamic_waypoint.getCurrentPosition());

    RCLCPP_INFO(
      this->get_logger(), "waypoint[%s] added: (%.2f, %.2f, %.2f)",
      dynamic_waypoint.getName().c_str(),
      dynamic_waypoint.getOriginalPosition().x(),
      dynamic_waypoint.getOriginalPosition().y(),
      dynamic_waypoint.getOriginalPosition().z());            // DEBUG
  }

  return true;
}

/** For faster waypoint modified */
void DynamicPolynomialTrajectoryGenerator::modifyWaypointCallback(
  const as2_msgs::msg::PoseStampedWithID::SharedPtr _msg)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Callback Waypoint[%s] to modify has been received",
    _msg->id.c_str());

  geometry_msgs::msg::PoseStamped pose_stamped = _msg->pose;
  if (_msg->pose.header.frame_id != desired_frame_id_) {
    try {
      pose_stamped = tf_handler_.convert(_msg->pose, desired_frame_id_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }
  }

  Eigen::Vector3d position;
  position.x() = pose_stamped.pose.position.x;
  position.y() = pose_stamped.pose.position.y;
  position.z() = pose_stamped.pose.position.z;
  trajectory_generator_->modifyWaypoint(_msg->id, position);
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
  paused_goal.header = goal_.header;
  paused_goal.header.stamp = this->now();
  paused_goal.yaw = goal_.yaw;
  paused_goal.max_speed = goal_.max_speed;

  paused_goal.path = std::vector<as2_msgs::msg::PoseWithID>();
  for (auto waypoint : goal_.path) {
    if (waypoint.id != feedback_.next_waypoint_id && !start_trajectory) {
      continue;
    }
    start_trajectory = true;
    paused_goal.path.push_back(waypoint);

    RCLCPP_INFO(
      this->get_logger(),
      "Waypoint remainings ID: %s, position : [%f, %f, %f]",
      waypoint.id.c_str(), waypoint.pose.position.x,
      waypoint.pose.position.y, waypoint.pose.position.z);
  }

  if (paused_goal.path.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "No waypoint remaining");
    return false;
  }

  // Generate vector of waypoints for trajectory generator, from goal to
  // dynamic_traj_generator::DynamicWaypoint::Vector
  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  waypoints_to_set.reserve(paused_goal.path.size() + 1);

  auto paused_goal_shared_ptr =
    std::make_shared<as2_msgs::action::GeneratePolynomialTrajectory::Goal>(
    paused_goal);
  if (!goalToDynamicWaypoint(paused_goal_shared_ptr, waypoints_to_set)) {
    return false;
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

  // Reset the trajectory generator
  trajectory_generator_ =
    std::make_shared<dynamic_traj_generator::DynamicTrajectory>();

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
    !first_run_)
  {
    result_msg->trajectory_generator_success = true;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  // Plot debug trajectory
  if (enable_debug_) {
    plotRefTrajPoint();
    if (trajectory_generator_->getWasTrajectoryRegenerated()) {
      RCLCPP_DEBUG(this->get_logger(), "Plot trajectory");
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
    if (!evaluateSetpoint(eval_time, setpoint)) {
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
  as2_msgs::msg::TrajectoryPoint & setpoint)
{
  dynamic_traj_generator::References traj_command;
  double yaw_angle;

  if (eval_time <= trajectory_generator_->getMinTime()) {
    eval_time = trajectory_generator_->getMinTime();
  } else if (eval_time >= trajectory_generator_->getMaxTime()) {
    eval_time = trajectory_generator_->getMaxTime();
  }
  bool succes_eval =
    trajectory_generator_->evaluateTrajectory(eval_time, traj_command);

  switch (yaw_mode_.mode) {
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

double DynamicPolynomialTrajectoryGenerator::computeYawAnglePathFacing(
  double vx, double vy)
{
  if (sqrt(vx * vx + vy * vy) > 0.1) {
    return as2::frame::getVector2DAngle(vx, vy);
  }
  return current_yaw_;
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
  nav_msgs::msg::Path path_msg;
  const float step = 0.2;
  const float max_time = trajectory_generator_->getMaxTime();
  const float min_time = trajectory_generator_->getMinTime();
  dynamic_traj_generator::References refs;
  const int n_measures = (max_time - min_time) / step;
  auto time_stamp = this->now();
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
  path_pub_->publish(path_msg);
}

void DynamicPolynomialTrajectoryGenerator::plotRefTrajPoint()
{
  visualization_msgs::msg::Marker point_msg;

  point_msg.header.frame_id = desired_frame_id_;
  point_msg.header.stamp = this->now();
  point_msg.type = visualization_msgs::msg::Marker::SPHERE;
  point_msg.action = visualization_msgs::msg::Marker::ADD;

  point_msg.color.r = 0.0f;
  point_msg.color.g = 0.0f;
  point_msg.color.b = 1.0f;
  point_msg.color.a = 1.0f;

  point_msg.scale.x = 0.2;
  point_msg.scale.y = 0.2;
  point_msg.scale.z = 0.2;

  point_msg.pose.position.x = trajectory_command_.setpoints[0].position.x;
  point_msg.pose.position.y = trajectory_command_.setpoints[0].position.y;
  point_msg.pose.position.z = trajectory_command_.setpoints[0].position.z;

  ref_point_pub->publish(point_msg);
}

/** Auxiliar Functions **/
void generateDynamicPoint(
  const as2_msgs::msg::PoseWithID & msg,
  dynamic_traj_generator::DynamicWaypoint & dynamic_point)
{
  dynamic_point.setName(msg.id);
  Eigen::Vector3d position;
  position.x() = msg.pose.position.x;
  position.y() = msg.pose.position.y;
  position.z() = msg.pose.position.z;
  dynamic_point.resetWaypoint(position);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(DynamicPolynomialTrajectoryGenerator)
