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
* @file gps_origin
*
* An state estimation plugin for setting the origin of the map frame using GPS
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#ifndef GPS_ORIGIN_HPP_
#define GPS_ORIGIN_HPP_

#include <tf2/LinearMath/Transform.h>
#include <utility>
#include <memory>
#include <geographic_msgs/msg/geo_point.hpp>

#include <as2_core/names/services.hpp>
#include <as2_core/utils/gps_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include <as2_msgs/srv/set_origin.hpp>

#include "as2_state_estimator/plugin_base.hpp"
#include "as2_core/names/topics.hpp"

namespace gps_origin
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Service<as2_msgs::srv::SetOrigin>::SharedPtr set_origin_srv_;
  rclcpp::Service<as2_msgs::srv::GetOrigin>::SharedPtr get_origin_srv_;

  bool set_origin_on_start_ = false;
  bool earth_to_map_set_ = false;
  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  double origin_alt_ = 0.0;

  geographic_msgs::msg::GeoPoint::UniquePtr origin_;
  sensor_msgs::msg::NavSatFix::UniquePtr gps_pose_;

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}
  void onSetup() override
  {
    node_ptr_->get_parameter("set_origin_on_start", set_origin_on_start_);

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

private:
  void generate_map_frame_from_gps(
    const geographic_msgs::msg::GeoPoint & origin,
    const sensor_msgs::msg::NavSatFix & gps_pose)
  {
    as2::gps::GpsHandler gps_handler;
    gps_handler.setOrigin(origin.latitude, origin.longitude, origin.altitude);
    double x, y, z;
    gps_handler.LatLon2Local(gps_pose.latitude, gps_pose.longitude, gps_pose.altitude, x, y, z);
    auto earth_to_map_ = tf2::Transform::getIdentity();
    earth_to_map_.setOrigin(tf2::Vector3(x, y, z));
    state_estimator_interface_->setEarthToMap(earth_to_map_, gps_pose.header.stamp, true);
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
        RCLCPP_WARN(node_ptr_->get_logger(), "be careful, using GPS pose as origin");
        RCLCPP_INFO(
          node_ptr_->get_logger(), "Origin set to %f, %f, %f", origin_lat_, origin_lon_,
          origin_alt_);
      }

      RCLCPP_INFO(
        node_ptr_->get_logger(), "GPS Callback: Map GPS pose set to %f, %f, %f",
        gps_pose_->latitude, gps_pose_->longitude, gps_pose_->altitude);

      generate_map_frame_from_gps(*origin_, *gps_pose_);
      earth_to_map_set_ = true;
    }
  }
};      // class GroundTruth
}       // namespace gps_origin
#endif  // GPS_ORIGIN_HPP_
