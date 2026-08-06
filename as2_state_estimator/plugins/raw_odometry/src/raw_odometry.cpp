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
* @file raw_odometry.cpp
*
* An state estimation plugin raw odometry for AeroStack2 implementation
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#include <string>
#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include "raw_odometry/raw_odometry.hpp"

namespace raw_odometry
{

void Plugin::onSetup()
{
  // Define odometry topic
  std::string odom_sub_topic;
  odom_sub_topic = getParameter<std::string>(node_ptr_, "raw_odometry.odom_sub_topic");

  if (odom_sub_topic == "") {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "No odometry topic provided, raw odometry plugin will not work");
    return;
  }

  RCLCPP_INFO(
    node_ptr_->get_logger(), "Using odometry topic: %s", odom_sub_topic.c_str());

  // Set earth to map from parameters
  set_earth_map_manually_ = getParameter<bool>(
    node_ptr_,
    "raw_odometry.earth_map_transform.set_earth_map");

  if (!set_earth_map_manually_) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Setting earth to map transform to identity (0, 0, 0)");
    // Set to identity (already initialized as identity)
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "Setting earth to map transform from parameters");
    double initial_x, initial_y, initial_z;
    initial_x = getParameter<double>(node_ptr_, "raw_odometry.earth_map_transform.position.x");
    initial_y = getParameter<double>(node_ptr_, "raw_odometry.earth_map_transform.position.y");
    initial_z = getParameter<double>(node_ptr_, "raw_odometry.earth_map_transform.position.z");
    double initial_roll, initial_pitch, initial_yaw;
    initial_roll = getParameter<double>(
      node_ptr_,
      "raw_odometry.earth_map_transform.orientation.roll");
    initial_pitch = getParameter<double>(
      node_ptr_,
      "raw_odometry.earth_map_transform.orientation.pitch");
    initial_yaw =
      getParameter<double>(node_ptr_, "raw_odometry.earth_map_transform.orientation.yaw");
    earth_to_map_.setOrigin(tf2::Vector3(initial_x, initial_y, initial_z));
    tf2::Quaternion q;
    q.setRPY(initial_roll, initial_pitch, initial_yaw);
    earth_to_map_.setRotation(q);
  }

  // GPS anchors the earth frame at a geodetic datum. It composes with the block above rather
  // than replacing it: with set_earth_map false the first fix also places map, with it true
  // earth_map_transform places map and GPS only supplies the datum served by get_origin.
  use_gps_ = getParameter<bool>(node_ptr_, "raw_odometry.use_gps");
  if (use_gps_) {
    setupGps();
  }

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

  if (!map_to_odom_set_) {
    geometry_msgs::msg::PoseWithCovariance map_to_odom = generateIdentityPose();
    state_estimator_interface_->setMapToOdomPose(map_to_odom, node_ptr_->now(), true);
    map_to_odom_set_ = true;
  }
}

void Plugin::setupGps()
{
  // Geodetic datum. Whichever source wins, the datum can only be established once, so the
  // set_origin service is refused afterwards.
  const std::string origin_mode =
    getParameter<std::string>(node_ptr_, "raw_odometry.gps_origin.set_origin");

  if (origin_mode == "manual") {
    origin_source_ = OriginSource::MANUAL;
    origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>();
    origin_->latitude =
      getParameter<double>(node_ptr_, "raw_odometry.gps_origin.coordinates.latitude");
    origin_->longitude =
      getParameter<double>(node_ptr_, "raw_odometry.gps_origin.coordinates.longitude");
    origin_->altitude =
      getParameter<double>(node_ptr_, "raw_odometry.gps_origin.coordinates.altitude");
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
        "Parameter <raw_odometry.gps_origin.set_origin> must be one of first_gps, manual or "
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

}  // namespace raw_odometry

PLUGINLIB_EXPORT_CLASS(raw_odometry::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)
