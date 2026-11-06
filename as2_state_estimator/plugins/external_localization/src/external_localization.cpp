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
* @file external_localization.cpp
*
* External localization state estimation plugin for AeroStack2 implementation
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*          Rodrigo Da Silva Gómez
*/

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <string>
#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include "external_localization/external_localization.hpp"

namespace external_localization
{

tf2::Transform interpolate(const tf2::Transform & from, const tf2::Transform & to, double ratio)
{
  tf2::Quaternion q_from = from.getRotation();
  tf2::Quaternion q_to = to.getRotation();
  if (q_from.dot(q_to) < 0.0) {q_to = -q_to;}  // same rotation, take the short way round
  return tf2::Transform(
    q_from.slerp(q_to, ratio).normalized(), from.getOrigin().lerp(to.getOrigin(), ratio));
}

tf2::Transform stepToward(
  const tf2::Transform & from, const tf2::Transform & to,
  double max_translation, double max_rotation)
{
  const double distance = (to.getOrigin() - from.getOrigin()).length();
  const double angle = from.getRotation().angleShortestPath(to.getRotation());
  double ratio = 1.0;
  if (distance > max_translation) {ratio = std::min(ratio, max_translation / distance);}
  if (angle > max_rotation) {ratio = std::min(ratio, max_rotation / angle);}
  return interpolate(from, to, ratio);
}

void Plugin::onSetup()
{
  // Define odometry topic
  std::string odom_sub_topic;
  odom_sub_topic = node_ptr_->getParameter<std::string>(param(odometry_topic_key_));

  if (odom_sub_topic == "") {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "No odometry topic provided, %s plugin will not work",
      prefix_.c_str());
    return;
  }

  RCLCPP_INFO(
    node_ptr_->get_logger(), "Using odometry topic: %s", odom_sub_topic.c_str());

  // Set earth to map from parameters
  set_earth_map_manually_ =
    node_ptr_->getParameter<bool>(param("earth_map_transform.set_earth_map"));

  if (!set_earth_map_manually_) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Setting earth to map transform to identity (0, 0, 0)");
    // Set to identity (already initialized as identity)
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "Setting earth to map transform from parameters");
    double initial_x, initial_y, initial_z;
    initial_x = node_ptr_->getParameter<double>(param("earth_map_transform.position.x"));
    initial_y = node_ptr_->getParameter<double>(param("earth_map_transform.position.y"));
    initial_z = node_ptr_->getParameter<double>(param("earth_map_transform.position.z"));
    double initial_roll, initial_pitch, initial_yaw;
    initial_roll =
      node_ptr_->getParameter<double>(param("earth_map_transform.orientation.roll"));
    initial_pitch =
      node_ptr_->getParameter<double>(param("earth_map_transform.orientation.pitch"));
    initial_yaw =
      node_ptr_->getParameter<double>(param("earth_map_transform.orientation.yaw"));
    earth_to_map_.setOrigin(tf2::Vector3(initial_x, initial_y, initial_z));
    tf2::Quaternion q;
    q.setRPY(initial_roll, initial_pitch, initial_yaw);
    earth_to_map_.setRotation(q);
  }

  // GPS anchors the earth frame at a geodetic datum. It composes with the block above rather
  // than replacing it: with set_earth_map false the first fix also places map, with it true
  // earth_map_transform places map and GPS only supplies the datum served by get_origin.
  use_gps_ = node_ptr_->getParameter<bool>(param("use_gps"));
  if (use_gps_) {
    setupGps();
  }

  setupLocalization();

  // Create subscription
  odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
    odom_sub_topic, as2_names::topics::sensor_measurements::qos,
    std::bind(&Plugin::odometryCallback, this, std::placeholders::_1));
}

std::vector<as2_state_estimator::TransformInformatonType>
Plugin::getTransformationTypesAvailable() const
{
  return {as2_state_estimator::TransformInformatonType::EARTH_TO_MAP,
    as2_state_estimator::TransformInformatonType::MAP_TO_ODOM,
    as2_state_estimator::TransformInformatonType::ODOM_TO_BASE,
    as2_state_estimator::TransformInformatonType::TWIST_IN_BASE};
}

void Plugin::setupTfTree()
{
  // Set earth to map from parameters if not set with topic. Under GPS this only applies when
  // earth_map_transform pins it; otherwise the GPS logic sets it from the first fix.
  if ((!use_gps_ || set_earth_map_manually_) && !earth_to_map_set_) {
    state_estimator_interface_->setEarthToMap(earth_to_map_, node_ptr_->now(), true);
    earth_to_map_set_ = true;
  }

  // With localization, map to odom moves and is published from the odometry callback instead
  if (!map_to_odom_set_ && !use_localization_) {
    geometry_msgs::msg::PoseWithCovariance map_to_odom = generateIdentityPose();
    state_estimator_interface_->setMapToOdomPose(map_to_odom, node_ptr_->now(), true);
    map_to_odom_set_ = true;
  }
}

void Plugin::setupLocalization()
{
  localization_topic_ =
    node_ptr_->getParameter<std::string>(param("localization.topic"), "");
  if (localization_topic_.empty()) {
    RCLCPP_INFO(
      node_ptr_->get_logger(), "No localization topic set, map to odom is a static identity");
    return;
  }
  use_localization_ = true;

  // Non-positive durations and rates make no sense, so they fall back to the default
  auto positive = [this](const std::string & key, double default_value) {
      const double value = node_ptr_->getParameter<double>(param(key), default_value);
      if (value > 0.0) {return value;}
      RCLCPP_WARN(
        node_ptr_->get_logger(), "Parameter %s must be positive, got %f. Using default %f",
        param(key).c_str(), value, default_value);
      return default_value;
    };
  odometry_history_ = positive("localization.odometry_history", odometry_history_);
  localization_timeout_ = positive("localization.timeout", localization_timeout_);
  max_rate_translation_ =
    positive("localization.jump.max_rate.translation", max_rate_translation_);
  max_rate_rotation_ = positive("localization.jump.max_rate.rotation", max_rate_rotation_);
  smoothing_time_ = positive("localization.jump.smoothing_time", smoothing_time_);
  warn_translation_ = positive("localization.jump.warn_translation", warn_translation_);
  wait_for_localization_ = node_ptr_->getParameter<bool>(
    param("localization.wait_for_localization"), wait_for_localization_);

  std::string mode =
    node_ptr_->getParameter<std::string>(param("localization.jump.mode"), "none");
  if (mode == "clamp") {
    jump_mode_ = JumpMode::CLAMP;
  } else if (mode == "smooth") {
    jump_mode_ = JumpMode::SMOOTH;
  } else {
    jump_mode_ = JumpMode::NONE;
    if (mode != "none") {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Parameter %s must be one of none, clamp or smooth, got '%s'. Using none",
        param("localization.jump.mode").c_str(), mode.c_str());
      mode = "none";
    }
  }

  RCLCPP_INFO(
    node_ptr_->get_logger(), "Localization from %s, jump mode %s. %s",
    localization_topic_.c_str(), mode.c_str(),
    wait_for_localization_ ? "self_localization is held until the first pose" :
    "map to odom is identity until the first pose");

  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
  localization_sub_ = node_ptr_->create_subscription<PoseMsg>(
    localization_topic_, as2_names::topics::sensor_measurements::qos,
    std::bind(&Plugin::localizationCallback, this, std::placeholders::_1));
}

void Plugin::setupGps()
{
  // Geodetic datum. Whichever source wins, the datum can only be established once, so the
  // set_origin service is refused afterwards.
  const std::string origin_mode =
    node_ptr_->getParameter<std::string>(param("gps_origin.set_origin"));

  if (origin_mode == "manual") {
    origin_source_ = OriginSource::MANUAL;
    origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>();
    origin_->latitude =
      node_ptr_->getParameter<double>(param("gps_origin.coordinates.latitude"));
    origin_->longitude =
      node_ptr_->getParameter<double>(param("gps_origin.coordinates.longitude"));
    origin_->altitude =
      node_ptr_->getParameter<double>(param("gps_origin.coordinates.altitude"));
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Origin set from parameters: %f, %f, %f",
      origin_->latitude, origin_->longitude, origin_->altitude);
  } else if (origin_mode == "service") {
    origin_source_ = OriginSource::SERVICE;
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Waiting for the set_origin service to provide the origin. GPS fixes are held aside "
      "until then, so earth to map is not published and the TF tree stays incomplete");
  } else {
    origin_source_ = OriginSource::FIRST_GPS;
    if (origin_mode != "first_gps") {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Parameter gps_origin.set_origin must be one of first_gps, manual or "
        "service, got '%s'. Using default (first_gps)", origin_mode.c_str());
    }
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Origin will be set by the first GPS fix received, or by the set_origin service if it "
      "is called before one arrives");
  }

  set_origin_srv_ = node_ptr_->create_service<as2_msgs::srv::SetOrigin>(
    as2_names::services::gps::set_origin,
    std::bind(&Plugin::setOriginCallback, this, std::placeholders::_1, std::placeholders::_2));
  get_origin_srv_ = node_ptr_->create_service<as2_msgs::srv::GetOrigin>(
    as2_names::services::gps::get_origin,
    std::bind(&Plugin::getOriginCallback, this, std::placeholders::_1, std::placeholders::_2));
  gps_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::NavSatFix>(
    as2_names::topics::sensor_measurements::gps, as2_names::topics::sensor_measurements::qos,
    std::bind(&Plugin::gpsCallback, this, std::placeholders::_1));
}

void Plugin::generateMapFrameFromGps(
  const geographic_msgs::msg::GeoPoint & origin,
  const sensor_msgs::msg::NavSatFix & gps_pose)
{
  if (set_earth_map_manually_) {
    // earth to map is pinned by earth_map_transform, so the fix must not move it. The datum
    // is still recorded, and still served by get_origin for GPS-referenced missions.
    RCLCPP_INFO_ONCE(
      node_ptr_->get_logger(),
      "Not placing map from the GPS fix: earth to map is set by earth_map_transform");
    return;
  }

  as2::gps::GpsHandler gps_handler;
  gps_handler.setOrigin(origin.latitude, origin.longitude, origin.altitude);
  double x, y, z;
  gps_handler.LatLon2Local(gps_pose.latitude, gps_pose.longitude, gps_pose.altitude, x, y, z);
  earth_to_map_.setOrigin(tf2::Vector3(x, y, z));
  state_estimator_interface_->setEarthToMap(earth_to_map_, gps_pose.header.stamp, true);
  earth_to_map_set_ = true;
}

void Plugin::getOriginCallback(
  const as2_msgs::srv::GetOrigin::Request::SharedPtr request,
  as2_msgs::srv::GetOrigin::Response::SharedPtr response)
{
  if (origin_) {
    response->origin = *origin_;
    response->success = true;
  } else {
    RCLCPP_WARN(node_ptr_->get_logger(), "Origin not set");
    response->success = false;
  }
}

void Plugin::setOriginCallback(
  const as2_msgs::srv::SetOrigin::Request::SharedPtr request,
  as2_msgs::srv::SetOrigin::Response::SharedPtr response)
{
  if (origin_) {
    RCLCPP_WARN(node_ptr_->get_logger(), "Origin already set");
    response->success = false;
    return;
  }

  origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>(request->origin);
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Origin set to %f, %f, %f",
    origin_->latitude, origin_->longitude, origin_->altitude);
  response->success = true;

  if (gps_pose_) {
    generateMapFrameFromGps(request->origin, *gps_pose_);
  } else {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Origin set via service; earth to map transform will be generated once a GPS fix arrives.");
  }
}

void Plugin::gpsCallback(sensor_msgs::msg::NavSatFix::UniquePtr msg)
{
  if (gps_pose_) {
    gps_sub_.reset();
    return;
  }
  gps_pose_ = std::move(msg);

  if (!origin_) {
    if (origin_source_ == OriginSource::SERVICE) {
      // Hold the fix aside. setOriginCallback consumes it as soon as the datum arrives, so
      // the service still works after the one and only fix this plugin ever reads.
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "GPS fix received and stored, waiting for the set_origin service to set the origin");
      return;
    }
    origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>();
    origin_->latitude = gps_pose_->latitude;
    origin_->longitude = gps_pose_->longitude;
    origin_->altitude = gps_pose_->altitude;
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Origin set from first GPS fix: %f, %f, %f",
      origin_->latitude, origin_->longitude, origin_->altitude);
  }

  generateMapFrameFromGps(*origin_, *gps_pose_);
}

void Plugin::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Odometry should have frame_id = odom and child_frame_id = base_link
  // We publish the transform from odom to base_link directly
  // The transforms from earth to map and map to odom will be identity transforms

  if (msg->header.frame_id != state_estimator_interface_->getOdomFrame()) {
    RCLCPP_WARN_ONCE(
      node_ptr_->get_logger(), "Received odometry in frame %s, expected %s. "
      "Frame id will be changed to expected one",
      msg->header.frame_id.c_str(), state_estimator_interface_->getOdomFrame().c_str());
  }

  if (msg->child_frame_id != state_estimator_interface_->getBaseFrame()) {
    RCLCPP_WARN_ONCE(
      node_ptr_->get_logger(),
      "Received odometry child_frame_id in frame %s, expected %s. "
      "Child frame id will be changed to expected one",
      msg->child_frame_id.c_str(), state_estimator_interface_->getBaseFrame().c_str());
  }

  // Setup tf tree if not already setup
  setupTfTree();

  if (use_localization_) {
    const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
    tf2::Transform odom_to_base;
    tf2::fromMsg(msg->pose.pose, odom_to_base);

    // Time going backwards (a restarted bag or simulation) invalidates the history
    if (!odom_history_.empty() && stamp < odom_history_.back().first) {
      odom_history_.clear();
    }
    odom_history_.emplace_back(stamp, odom_to_base);
    while (odom_history_.size() > 1 &&
      (stamp - odom_history_.front().first).seconds() > odometry_history_)
    {
      odom_history_.pop_front();
    }

    const double dt =
      last_odom_stamp_.nanoseconds() == 0 ? 0.0 : (stamp - last_odom_stamp_).seconds();
    last_odom_stamp_ = stamp;

    if (localized_) {
      advanceMapToOdom(dt);
      const double since_last = (node_ptr_->now() - last_localization_time_).seconds();
      if (since_last > localization_timeout_) {
        RCLCPP_ERROR_THROTTLE(
          node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
          "No localization on %s for %.1f s. map to odom is frozen and the drone is flying "
          "on odometry alone", localization_topic_.c_str(), since_last);
      }
    }

    if (localized_ || !wait_for_localization_) {
      state_estimator_interface_->setMapToOdomPose(map_to_odom_, msg->header.stamp, false);
      map_to_odom_set_ = true;
    } else {
      RCLCPP_INFO_THROTTLE(
        node_ptr_->get_logger(), *node_ptr_->get_clock(), 5000,
        "Waiting for localization on %s, self_localization is held until the first pose",
        localization_topic_.c_str());
    }
  }

  // Extract pose with covariance from odometry
  geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
  pose_with_covariance.pose = msg->pose.pose;
  pose_with_covariance.covariance = msg->pose.covariance;

  // Publish odom to base_link pose
  state_estimator_interface_->setOdomToBaseLinkPose(pose_with_covariance, msg->header.stamp);

  // Extract twist with covariance from odometry
  geometry_msgs::msg::TwistWithCovariance twist_with_covariance;
  twist_with_covariance.twist = msg->twist.twist;
  twist_with_covariance.covariance = msg->twist.covariance;

  // Publish twist in base frame
  state_estimator_interface_->setTwistInBaseFrame(twist_with_covariance, msg->header.stamp);
}

void Plugin::localizationCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const std::string & frame = msg->header.frame_id;
  tf2::Transform pose;
  tf2::fromMsg(msg->pose.pose, pose);

  tf2::Transform map_to_base;
  if (frame == state_estimator_interface_->getMapFrame()) {
    map_to_base = pose;
  } else if (frame == state_estimator_interface_->getEarthFrame()) {
    if (!earth_to_map_set_) {
      RCLCPP_WARN_THROTTLE(
        node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
        "Localization pose in %s dropped, earth to map is not set yet", frame.c_str());
      return;
    }
    map_to_base = earth_to_map_.inverse() * pose;
  } else {
    RCLCPP_ERROR_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
      "Localization pose in frame '%s' dropped, expected '%s' or '%s'", frame.c_str(),
      state_estimator_interface_->getMapFrame().c_str(),
      state_estimator_interface_->getEarthFrame().c_str());
    return;
  }

  if (odom_history_.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
      "Localization pose dropped, no odometry received yet");
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
  const auto odom_to_base = odomToBaseAt(stamp);
  if (!odom_to_base) {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
      "Localization pose dropped, it is %.3f s older than the newest odometry and the history "
      "keeps %.1f s. Increase %s", (odom_history_.back().first - stamp).seconds(),
      odometry_history_, param("localization.odometry_history").c_str());
    return;
  }

  map_to_odom_target_ = map_to_base * odom_to_base->inverse();

  if (!localized_ && wait_for_localization_) {
    // Nothing was published yet, so the first correction is the starting point, not a jump
    map_to_odom_ = map_to_odom_target_;
  } else {
    const double step = (map_to_odom_target_.getOrigin() - map_to_odom_.getOrigin()).length();
    if (step > warn_translation_) {
      RCLCPP_WARN(
        node_ptr_->get_logger(), "Localization correction of %.2f m, above %s", step,
        param("localization.jump.warn_translation").c_str());
    }
    if (jump_mode_ == JumpMode::NONE) {
      map_to_odom_ = map_to_odom_target_;
    }
  }

  if (!localized_) {
    const tf2::Vector3 & t = map_to_odom_target_.getOrigin();
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "First localization pose received, map to odom offset x %.2f y %.2f z %.2f yaw %.2f",
      t.x(), t.y(), t.z(), tf2::getYaw(map_to_odom_target_.getRotation()));
  }
  localized_ = true;
  last_localization_time_ = node_ptr_->now();
}

std::optional<tf2::Transform> Plugin::odomToBaseAt(const rclcpp::Time & stamp) const
{
  if (odom_history_.empty() || stamp < odom_history_.front().first) {
    return std::nullopt;
  }
  // shortcut: a pose newer than the last odometry uses it as is, fine while odometry is faster
  // than localization. Extrapolate if a setup ever has it the other way round.
  if (stamp >= odom_history_.back().first) {
    return odom_history_.back().second;
  }
  const auto after = std::lower_bound(
    odom_history_.begin(), odom_history_.end(), stamp,
    [](const auto & sample, const rclcpp::Time & t) {return sample.first < t;});
  const auto before = std::prev(after);
  const double span = (after->first - before->first).seconds();
  const double ratio = span > 0.0 ? (stamp - before->first).seconds() / span : 1.0;
  return interpolate(before->second, after->second, ratio);
}

void Plugin::advanceMapToOdom(double dt)
{
  if (dt <= 0.0) {return;}
  switch (jump_mode_) {
    case JumpMode::NONE:
      break;  // the localization callback already applied the target
    case JumpMode::CLAMP:
      map_to_odom_ = stepToward(
        map_to_odom_, map_to_odom_target_, max_rate_translation_ * dt, max_rate_rotation_ * dt);
      break;
    case JumpMode::SMOOTH:
      map_to_odom_ = interpolate(
        map_to_odom_, map_to_odom_target_, 1.0 - std::exp(-dt / smoothing_time_));
      break;
  }
}

}  // namespace external_localization

PLUGINLIB_EXPORT_CLASS(
  external_localization::Plugin,
  as2_state_estimator_plugin_base::StateEstimatorBase)
