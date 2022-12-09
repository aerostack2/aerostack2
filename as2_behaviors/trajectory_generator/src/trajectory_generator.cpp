/*!*******************************************************************************************
 *  \file       trajectory_generator.cpp
 *  \brief      trajectory_generator implementation file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "trajectory_generator.hpp"

#include <rclcpp/qos.hpp>

TrajectoryGenerator::TrajectoryGenerator()
    : as2::Node("trajectory_generator"),
      v_positions_(4),
      v_velocities_(4),
      v_accelerations_(4),
      motion_handler(this),
      eval_time_(0, 0),
      tf_handler_(this) {
  set_trajectory_waypoints_srv_ =
      this->create_service<as2_msgs::srv::SendTrajectoryWaypoints>(
          as2_names::services::motion_reference::send_traj_wayp,
          std::bind(
              &TrajectoryGenerator::setTrajectoryWaypointsSrvCall, this,
              std::placeholders::_1,  // Corresponds to the 'request'  input
              std::placeholders::_2   // Corresponds to the 'response' input
              ));

  add_trajectory_waypoints_srv_ =
      this->create_service<as2_msgs::srv::SendTrajectoryWaypoints>(
          as2_names::services::motion_reference::add_traj_wayp,
          std::bind(
              &TrajectoryGenerator::addTrajectoryWaypointsSrvCall, this,
              std::placeholders::_1,  // Corresponds to the 'request'  input
              std::placeholders::_2   // Corresponds to the 'response' input
              ));

  set_speed_srv_ = this->create_service<as2_msgs::srv::SetSpeed>(
      as2_names::services::motion_reference::set_traj_speed,
      std::bind(&TrajectoryGenerator::setSpeedSrvCall, this,
                std::placeholders::_1,  // Corresponds to the 'request'  input
                std::placeholders::_2   // Corresponds to the 'response' input
                ));

  run_node_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "traj_gen/run_node",
      std::bind(&TrajectoryGenerator::runNodeSrvCall, this,
                std::placeholders::_1,  // Corresponds to the 'request'  input
                std::placeholders::_2   // Corresponds to the 'response' input
                ));

  mod_waypoint_sub_ =
      this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
          as2_names::topics::motion_reference::modify_waypoint,
          as2_names::topics::motion_reference::qos_waypoint,
          std::bind(&TrajectoryGenerator::modifyWaypointCallback, this,
                    std::placeholders::_1));

  yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "traj_gen/yaw", rclcpp::SensorDataQoS(),
      std::bind(&TrajectoryGenerator::yawCallback, this,
                std::placeholders::_1));

  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist,
      as2_names::topics::self_localization::qos,
      std::bind(&TrajectoryGenerator::state_callback, this,
                std::placeholders::_1));

  ref_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      REF_TRAJ_TOPIC, 1);

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(PATH_DEBUG_TOPIC, 1);

  traj_gen_info_pub_ = this->create_publisher<as2_msgs::msg::TrajGenInfo>(
      as2_names::topics::motion_reference::traj_gen_info, 1);

  static auto control_timer_ = this->create_timer(
      std::chrono::milliseconds(100),
      std::bind(&TrajectoryGenerator::publishTrajGenInfo, this));

  odom_frame_id_ = as2::tf::generateTfName(this, "odom");
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  traj_gen_info_msg_.active_status = as2_msgs::msg::TrajGenInfo::STOPPED;
}

void TrajectoryGenerator::publishTrajGenInfo() {
  traj_gen_info_msg_.header.stamp = this->now();
  traj_gen_info_msg_.node_status.status = as2_msgs::msg::NodeStatus::ACTIVE;
  traj_gen_info_pub_->publish(traj_gen_info_msg_);
}

void TrajectoryGenerator::setup() {
  trajectory_generator_ =
      std::make_shared<dynamic_traj_generator::DynamicTrajectory>();
  has_yaw_from_topic_ = false;
  first_time_ = true;
  time_zero_ = this->now();
  eval_time_ = time_zero_ - time_zero_;
  publish_trajectory_ = false;
  evaluate_trajectory_ = false;
  has_odom_ = false;
  has_prev_v_ = false;
  prev_vx_ = 0.0;
  prev_vy_ = 0.0;
  yaw_mode_ = 0;
  begin_traj_yaw_ = 0.0f;
  v_positions_ = {0.0f, 0.0f, 0.0f, 0.0f};
  v_velocities_ = {0.0f, 0.0f, 0.0f, 0.0f};
  v_accelerations_ = {0.0f, 0.0f, 0.0f, 0.0f};
  traj_gen_info_msg_.active_status = as2_msgs::msg::TrajGenInfo::WAITING;
}

void TrajectoryGenerator::run() {
  // static bool first_time = true;
  // static rclcpp::Time time_zero = rclcpp::Clock().now();
  // static auto eval_time = time_zero - time_zero;
  // static bool publish_trajectory = false;

  if (!evaluate_trajectory_ || !trajectory_generator_) {
    return;
  }
  if (!has_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry information available");
    return;
  }
  if (first_time_) {
    publish_trajectory_ = evaluateTrajectory(0);
    if (publish_trajectory_) first_time_ = false;
    time_zero_ = this->now();
  } else {
    eval_time_ = this->now() - time_zero_;

    if (trajectory_generator_->getMaxTime() + 0.2 > eval_time_.seconds()) {
      traj_gen_info_msg_.active_status = as2_msgs::msg::TrajGenInfo::EVALUATING;
      publish_trajectory_ = evaluateTrajectory(eval_time_.seconds());
    } else {
      traj_gen_info_msg_.active_status = as2_msgs::msg::TrajGenInfo::WAITING;
      evaluateTrajectory(eval_time_.seconds());
      publish_trajectory_ = false;
    }
  }

  plotRefTrajPoint();

  if (publish_trajectory_) {
    motion_handler.sendTrajectoryCommandWithYawAngle(
        odom_frame_id_, v_positions_, v_velocities_, v_accelerations_);
    if (trajectory_generator_->getWasTrajectoryRegenerated()) {
      RCLCPP_DEBUG(this->get_logger(), "Plot trajectory");
      plotTrajectory();
    }
  }
}

bool TrajectoryGenerator::evaluateTrajectory(double _eval_time) {
  bool publish_trajectory = false;
  publish_trajectory =
      trajectory_generator_->evaluateTrajectory(_eval_time, references_);

  for (int i = 0; i < 3; i++) {
    v_positions_[i] = references_.position[i];
    v_velocities_[i] = references_.velocity[i];
    v_accelerations_[i] = references_.acceleration[i];
  }

  switch (yaw_mode_) {
    case as2_msgs::msg::TrajectoryWaypoints::KEEP_YAW: {
      v_positions_[3] = begin_traj_yaw_;
    } break;

    case as2_msgs::msg::TrajectoryWaypoints::PATH_FACING: {
      if (!has_prev_v_) {
        prev_vx_ = references_.velocity.x();
        prev_vy_ = references_.velocity.y();
        has_prev_v_ = true;
      }
      if (fabs(references_.velocity.x()) > 0.01 ||
          (references_.velocity.y()) > 0.01) {
        v_positions_[3] = atan2f((double)references_.velocity.y(),
                                 (double)references_.velocity.x());
        prev_vx_ = references_.velocity.x();
        prev_vy_ = references_.velocity.y();
      } else {
        v_positions_[3] = atan2f((double)prev_vy_, (double)prev_vx_);
      }
    } break;

    case as2_msgs::msg::TrajectoryWaypoints::GENERATE_YAW_TRAJ: {
      v_positions_[3] = 0.0f;
      RCLCPP_ERROR(get_logger(), "YAW MODE NOT IMPLEMENTED YET");
    } break;

    case as2_msgs::msg::TrajectoryWaypoints::YAW_FROM_TOPIC: {
      if (has_yaw_from_topic_) {
        v_positions_[3] = yaw_from_topic_;
        // RCLCPP_DEBUG(get_logger(), "RECEIVED YAW %.2f", yaw_from_topic_);
      } else {
        v_positions_[3] = begin_traj_yaw_;
        // RCLCPP_DEBUG(get_logger(), "INITIAL YAW %.2f", begin_traj_yaw_);
      }
    } break;

    default: {
      RCLCPP_ERROR(get_logger(), "YAW MODE NOT DEFINED");
    } break;
  }

  return publish_trajectory;
}

void TrajectoryGenerator::updateState() {
  if (!trajectory_generator_) {
    return;
  }
  Eigen::Vector3d current_position(current_state_pose_.pose.position.x,
                                   current_state_pose_.pose.position.y,
                                   current_state_pose_.pose.position.z);
  trajectory_generator_->updateVehiclePosition(current_position);
}

/************************/
/** Services Callbacks **/
/************************/

void TrajectoryGenerator::setTrajectoryWaypointsSrvCall(
    const std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Request>
        _request,
    std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Response>
        _response) {
  if (!trajectory_generator_) {
    RCLCPP_WARN(
        this->get_logger(),
        "No trajectory generator available start trajectory generator first");
    _response->success = false;
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Waypoints to set has been received");
  _response->success = true;
  auto &waypoints_msg = *(_request.get());

  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  yaw_mode_ = _request->waypoints.yaw_mode;

  waypoints_to_set.reserve(waypoints_msg.waypoints.poses.size());

  for (auto waypoint : waypoints_msg.waypoints.poses) {
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    generateDynamicPoint(waypoint, dynamic_waypoint);
    waypoints_to_set.emplace_back(dynamic_waypoint);
  }

  begin_traj_yaw_ = extractYawFromQuat(current_state_pose_.pose.orientation);
  trajectory_generator_->setWaypoints(waypoints_to_set);
  evaluate_trajectory_ = true;
}

void TrajectoryGenerator::addTrajectoryWaypointsSrvCall(
    const std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Request>
        _request,
    std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Response>
        _response) {
  RCLCPP_DEBUG(this->get_logger(), "Waypoints to add has been received");
  if (!trajectory_generator_) {
    RCLCPP_WARN(
        this->get_logger(),
        "No trajectory generator available start trajectory generator first");
    _response->success = false;
  }
  _response->success = true;

  dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;

  for (auto waypoint : _request->waypoints.poses) {
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    generateDynamicPoint(waypoint, dynamic_waypoint);
    trajectory_generator_->appendWaypoint(dynamic_waypoint);
    RCLCPP_INFO(this->get_logger(), "waypoint[%s] added: (%.2f, %.2f, %.2f)",
                dynamic_waypoint.getName().c_str(),
                dynamic_waypoint.getOriginalPosition().x(),
                dynamic_waypoint.getOriginalPosition().y(),
                dynamic_waypoint.getOriginalPosition().z());  // DEBUG
  }
}

void TrajectoryGenerator::setSpeedSrvCall(
    const std::shared_ptr<as2_msgs::srv::SetSpeed::Request> _request,
    std::shared_ptr<as2_msgs::srv::SetSpeed::Response> _response) {
  if (!trajectory_generator_) {
    RCLCPP_WARN(
        this->get_logger(),
        "No trajectory generator available start trajectory generator first");
    _response->success = false;
    return;
  }
  float max_speed = _request->speed.speed;
  RCLCPP_INFO(this->get_logger(), "Speed (%.2f) to be set has been received",
              max_speed);
  if (max_speed <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Speed must be > 0.0 m/s");
    _response->success = false;
    return;
  }
  _response->success = true;

  trajectory_generator_->setSpeed(max_speed);
}

void TrajectoryGenerator::runNodeSrvCall(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> _request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> _response) {
  if (_request->data) {
    if (traj_gen_info_msg_.active_status !=
        as2_msgs::msg::TrajGenInfo::STOPPED) {
      RCLCPP_INFO(this->get_logger(),
                  "Trajectory generator is already running");
      _response->success = true;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator node is running");
    setup();
    _response->success = true;
  } else {
    stop();
    RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator node is stopped");
    _response->success = true;
  }
};

/*********************/
/** Topic Callbacks **/
/*********************/

void TrajectoryGenerator::modifyWaypointCallback(
    const as2_msgs::msg::PoseStampedWithID::SharedPtr _msg) {
  RCLCPP_DEBUG(this->get_logger(), "Waypoint[%s] to modify has been received",
               _msg->id.c_str());
  if (!trajectory_generator_) {
    RCLCPP_WARN(
        this->get_logger(),
        "No trajectory generator available start trajectory generator first");
  }
  dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
  Eigen::Vector3d position;
  position.x() = _msg->pose.position.x;
  position.y() = _msg->pose.position.y;
  position.z() = _msg->pose.position.z;
  trajectory_generator_->modifyWaypoint(_msg->id, position);
}

// TODO: if onExecute is done with timer no atomic attributes needed
void TrajectoryGenerator::state_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
  if (!has_odom_) {
    RCLCPP_INFO(this->get_logger(), "State callback working");
    has_odom_ = true;
  }

  try {
    auto [pose_msg, twist_msg] = tf_handler_.getState(
        *_twist_msg, odom_frame_id_, odom_frame_id_, base_link_frame_id_);

    current_state_pose_ = pose_msg;
    current_state_twist_ = twist_msg;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    return;
  }

  updateState();
};

void TrajectoryGenerator::waypointsCallback(
    const as2_msgs::msg::TrajectoryWaypoints::SharedPtr _msg) {
  if (!trajectory_generator_) {
    RCLCPP_WARN(
        this->get_logger(),
        "No trajectory generator available start trajectory generator first");
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Waypoints received");
  auto &waypoints_msg = *(_msg.get());

  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  waypoints_to_set.reserve(waypoints_msg.poses.size());

  float max_speed = waypoints_msg.max_speed;
  yaw_mode_ = waypoints_msg.yaw_mode;

  if (max_speed <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Speed must be > 0.0 m/s");
    return;
  }

  trajectory_generator_->setSpeed(max_speed);

  for (auto waypoint : waypoints_msg.poses) {
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    generateDynamicPoint(waypoint, dynamic_waypoint);
    waypoints_to_set.emplace_back(dynamic_waypoint);
  }

  // DEBUG
  for (auto &dw : waypoints_to_set) {
    RCLCPP_INFO(this->get_logger(), "setting waypoint[%s]: (%.2f, %.2f, %.2f)",
                dw.getName().c_str(), dw.getOriginalPosition().x(),
                dw.getOriginalPosition().y(), dw.getOriginalPosition().z());
  }

  trajectory_generator_->setWaypoints(waypoints_to_set);
  evaluate_trajectory_ = true;
}

void TrajectoryGenerator::yawCallback(
    const std_msgs::msg::Float32::SharedPtr _msg) {
  has_yaw_from_topic_ = true;
  yaw_from_topic_ = _msg->data;
}

/********************/
/** Debug Funtions **/
/********************/

void TrajectoryGenerator::plotTrajectory() {
  // launch async plot
  if (plot_thread_.joinable()) {
    plot_thread_.join();
  }
  plot_thread_ = std::thread(&TrajectoryGenerator::plotTrajectoryThread, this);
}

void TrajectoryGenerator::plotTrajectoryThread() {
  nav_msgs::msg::Path path_msg;
  const float step = 0.2;
  const float max_time = trajectory_generator_->getMaxTime();
  const float min_time = trajectory_generator_->getMinTime();
  dynamic_traj_generator::References refs;
  const int n_measures = (max_time - min_time) / step;
  auto time_stamp = rclcpp::Clock().now();
  path_msg.poses.reserve(n_measures);
  for (float time = min_time; time <= max_time; time += step) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = odom_frame_id_;
    pose_msg.header.stamp = time_stamp;
    trajectory_generator_->evaluateTrajectory(time, refs, true, true);
    pose_msg.pose.position.x = refs.position.x();
    pose_msg.pose.position.y = refs.position.y();
    pose_msg.pose.position.z = refs.position.z();
    path_msg.poses.emplace_back(pose_msg);
  }
  path_msg.header.frame_id = odom_frame_id_;
  path_msg.header.stamp = time_stamp;

  RCLCPP_INFO(this->get_logger(), "DEBUG: Plotting trajectory");
  path_pub_->publish(path_msg);
}

void TrajectoryGenerator::plotRefTrajPoint() {
  visualization_msgs::msg::Marker point_msg;

  point_msg.header.frame_id = odom_frame_id_;
  point_msg.header.stamp = rclcpp::Clock().now();
  point_msg.type = visualization_msgs::msg::Marker::SPHERE;
  point_msg.action = visualization_msgs::msg::Marker::ADD;

  point_msg.color.r = 0.0f;
  point_msg.color.g = 0.0f;
  point_msg.color.b = 1.0f;
  point_msg.color.a = 1.0f;

  point_msg.scale.x = 0.2;
  point_msg.scale.y = 0.2;
  point_msg.scale.z = 0.2;

  point_msg.pose.position.x = v_positions_[0];
  point_msg.pose.position.y = v_positions_[1];
  point_msg.pose.position.z = v_positions_[2];

  ref_point_pub->publish(point_msg);
}

/*******************/
/** Aux Functions **/
/*******************/

double extractYawFromQuat(const geometry_msgs::msg::Quaternion &quat) {
  double roll, pitch, yaw;
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 R(q);
  R.getRPY(roll, pitch, yaw);
  return yaw;
};

void generateDynamicPoint(
    const as2_msgs::msg::PoseStampedWithID &msg,
    dynamic_traj_generator::DynamicWaypoint &dynamic_point) {
  dynamic_point.setName(msg.id);
  Eigen::Vector3d position;
  position.x() = msg.pose.position.x;
  position.y() = msg.pose.position.y;
  position.z() = msg.pose.position.z;
  dynamic_point.resetWaypoint(position);
}

void generateDynamicPoint(
    const geometry_msgs::msg::PoseStamped &msg,
    dynamic_traj_generator::DynamicWaypoint &dynamic_point) {
  Eigen::Vector3d position;
  position.x() = msg.pose.position.x;
  position.y() = msg.pose.position.y;
  position.z() = msg.pose.position.z;
  dynamic_point.resetWaypoint(position);
}

void generateDynamicPoint(
    const nav_msgs::msg::Odometry &msg,
    dynamic_traj_generator::DynamicWaypoint &dynamic_point) {
  Eigen::Vector3d position;
  position.x() = msg.pose.pose.position.x;
  position.y() = msg.pose.pose.position.y;
  position.z() = msg.pose.pose.position.z;
  dynamic_point.resetWaypoint(position);
}
