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
* @file raw_odometry.hpp
*
* An state estimation plugin external odom for AeroStack2
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#ifndef RAW_ODOMETRY_HPP_
#define RAW_ODOMETRY_HPP_

#include <string>
#include <utility>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <as2_core/names/services.hpp>
#include <as2_core/utils/gps_utils.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include <as2_msgs/srv/set_origin.hpp>

#include "as2_state_estimator/plugin_base.hpp"

namespace raw_odometry
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  rclcpp::Service<as2_msgs::srv::SetOrigin>::SharedPtr set_origin_srv_;
  rclcpp::Service<as2_msgs::srv::GetOrigin>::SharedPtr get_origin_srv_;

  bool use_gps_ = false;
  bool set_origin_on_start_ = false;
  double earth_to_map_height_ = 0.0;
  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  double origin_alt_ = 0.0;

  geometry_msgs::msg::TransformStamped earth_to_map_;
  geographic_msgs::msg::GeoPoint::UniquePtr origin_;
  sensor_msgs::msg::NavSatFix::UniquePtr gps_pose_;

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}

  void on_setup() override
  {
    std::string odom_topic = as2_names::topics::sensor_measurements::odom;
    node_ptr_->get_parameter("odom_topic", odom_topic);
    odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, as2_names::topics::sensor_measurements::qos,
      std::bind(&Plugin::odom_callback, this, std::placeholders::_1));

    node_ptr_->get_parameter("use_gps", use_gps_);

    node_ptr_->get_parameter("set_origin_on_start", set_origin_on_start_);

    // publish static transform from earth to map and map to odom
    geometry_msgs::msg::TransformStamped map_to_odom =
      as2::tf::getTransformation(get_map_frame(), get_odom_frame(), 0, 0, 0, 0, 0, 0);
    earth_to_map_ =
      as2::tf::getTransformation(get_earth_frame(), get_map_frame(), 0, 0, 0, 0, 0, 0);
    publish_static_transform(map_to_odom);

    if (!use_gps_) {
      // TODO(javilinos): MODIFY this to a initial earth to map transform (reading initial position
      // from parameters or msgs )
      earth_to_map_ =
        as2::tf::getTransformation(get_earth_frame(), get_map_frame(), 0, 0, 0, 0, 0, 0);
      publish_static_transform(earth_to_map_);
    } else {
      set_origin_srv_ = node_ptr_->create_service<as2_msgs::srv::SetOrigin>(
        as2_names::services::gps::set_origin,
        std::bind(
          &Plugin::setOriginCallback, this, std::placeholders::_1,
          std::placeholders::_2));
      get_origin_srv_ = node_ptr_->create_service<as2_msgs::srv::GetOrigin>(
        as2_names::services::gps::get_origin,
        std::bind(
          &Plugin::getOriginCallback, this, std::placeholders::_1,
          std::placeholders::_2));
      gps_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::NavSatFix>(
        as2_names::topics::sensor_measurements::gps, as2_names::topics::sensor_measurements::qos,
        std::bind(&Plugin::gps_callback, this, std::placeholders::_1));

      if (node_ptr_->has_parameter("earth_to_map_height")) {
        node_ptr_->get_parameter("earth_to_map_height", earth_to_map_height_);
        RCLCPP_INFO(node_ptr_->get_logger(), "Earth to map height set to %f", earth_to_map_height_);
      }

      if (set_origin_on_start_ && node_ptr_->has_parameter("set_origin.lat") &&
        node_ptr_->has_parameter("set_origin.lon") &&
        node_ptr_->has_parameter("set_origin.alt"))
      {
        node_ptr_->get_parameter("set_origin.lat", origin_lat_);
        node_ptr_->get_parameter("set_origin.lon", origin_lon_);
        node_ptr_->get_parameter("set_origin.alt", origin_alt_);

        origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>();
        origin_->latitude = origin_lat_;
        origin_->longitude = origin_lon_;
        origin_->altitude = origin_alt_;

        RCLCPP_INFO(
          node_ptr_->get_logger(), "Origin set to %f, %f, %f", origin_lat_, origin_lon_,
          origin_alt_);
      } else {
        RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for origin to be set");
      }
    }
  }

private:
  void generate_map_frame_from_gps(
    const geographic_msgs::msg::GeoPoint & origin,
    const sensor_msgs::msg::NavSatFix & gps_pose)
  {
    as2::gps::GpsHandler gps_handler;
    gps_handler.setOrigin(origin.latitude, origin.longitude, origin.altitude);
    double x, y, z;
    gps_handler.LatLon2Local(gps_pose.latitude, gps_pose.longitude, gps_pose.altitude, x, y, z);
    if (!node_ptr_->has_parameter("earth_to_map_height")) {
      earth_to_map_height_ = gps_pose.altitude - origin.altitude;
    }
    earth_to_map_ = as2::tf::getTransformation(
      get_earth_frame(), get_map_frame(), x, y,
      earth_to_map_height_, 0, 0, 0);

    publish_static_transform(earth_to_map_);
  }

  void odom_callback(const nav_msgs::msg::Odometry::UniquePtr msg)
  {
    // odom should have frame_id = odom and child_frame_id = base_link
    // since we only have this message for generating the tf tree we will publish the transform
    // from odom to base_link directly and the transform from earth to map and map to odom  will
    // be the identity transform
    if (msg->header.frame_id != get_odom_frame()) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Received odom in frame %s, expected %s",
        msg->header.frame_id.c_str(), get_odom_frame().c_str());
      return;
    }
    if (msg->child_frame_id != get_base_frame()) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Received odom child_frame_id  in frame %s, expected %s",
        msg->child_frame_id.c_str(), get_base_frame().c_str());
      return;
    }

    auto transform = geometry_msgs::msg::TransformStamped();
    transform.header = msg->header;
    transform.child_frame_id = msg->child_frame_id;
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation = msg->pose.pose.orientation;

    publish_transform(transform);  // publish transform from odom to base_link

    tf2::fromMsg(transform.transform, odom_to_baselink);
    tf2::fromMsg(earth_to_map_.transform, earth_to_map);

    convert_odom_to_baselink_2_earth_to_baselink_transform(
      odom_to_baselink, earth_to_baselink,
      earth_to_map);

    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.frame_id = get_earth_frame();
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = earth_to_baselink.getOrigin().x();
    pose.pose.position.y = earth_to_baselink.getOrigin().y();
    pose.pose.position.z = earth_to_baselink.getOrigin().z();
    pose.pose.orientation = tf2::toMsg(earth_to_baselink.getRotation());
    publish_pose(pose);
    // publish twist in "base_link" frame
    auto twist = geometry_msgs::msg::TwistStamped();
    twist.header.frame_id = get_base_frame();
    twist.header.stamp = msg->header.stamp;
    twist.twist = msg->twist.twist;
    publish_twist(twist);
  }

  void getOriginCallback(
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

  void setOriginCallback(
    const as2_msgs::srv::SetOrigin::Request::SharedPtr request,
    as2_msgs::srv::SetOrigin::Response::SharedPtr response)
  {
    if (origin_) {
      RCLCPP_WARN(node_ptr_->get_logger(), "Origin already set");
      response->success = false;
    } else {
      origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>(request->origin);
      RCLCPP_INFO(
        node_ptr_->get_logger(), "Origin set to %f, %f, %f", origin_->latitude,
        origin_->longitude, origin_->altitude);
      response->success = true;
      generate_map_frame_from_gps(request->origin, *gps_pose_);
    }
  }

  void gps_callback(sensor_msgs::msg::NavSatFix::UniquePtr msg)
  {
    // This sould only be called when the use_gps_origin is true

    if (gps_pose_) {
      gps_sub_.reset();
      return;
    }

    gps_pose_ = std::move(msg);

    if (set_origin_on_start_) {
      if (!origin_) {
        origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>();
        origin_->latitude = gps_pose_->latitude;
        origin_->longitude = gps_pose_->longitude;
        origin_->altitude = gps_pose_->altitude;
        RCLCPP_WARN(node_ptr_->get_logger(), "Careful, using GPS pose as origin");
        RCLCPP_INFO(
          node_ptr_->get_logger(), "Origin set to %f, %f, %f", origin_lat_, origin_lon_,
          origin_alt_);
      }

      RCLCPP_INFO(
        node_ptr_->get_logger(), "GPS Callback: Map GPS pose set to %f, %f, %f",
        gps_pose_->latitude, gps_pose_->longitude, gps_pose_->altitude);

      generate_map_frame_from_gps(*origin_, *gps_pose_);
    }
  }
};

}  // namespace raw_odometry

#endif  // RAW_ODOMETRY_HPP_
