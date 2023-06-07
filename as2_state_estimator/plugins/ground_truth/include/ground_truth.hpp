/*!*******************************************************************************************
 *  \file       ground_truth.hpp
 *  \brief      An state estimation plugin ground truth for AeroStack2
 *  \authors    Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *              Pedro Arias Pérez
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

#ifndef __GROUND_TRUTH_H__
#define __GROUND_TRUTH_H__

#include <as2_core/names/services.hpp>
#include <as2_core/utils/gps_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include <as2_msgs/srv/set_origin.hpp>
#include <as2_state_estimator/plugin_base.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <regex>

namespace ground_truth {

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase {
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  rclcpp::Service<as2_msgs::srv::SetOrigin>::SharedPtr set_origin_srv_;
  rclcpp::Service<as2_msgs::srv::GetOrigin>::SharedPtr get_origin_srv_;

  bool use_gps_             = false;
  bool set_origin_on_start_ = false;
  bool origin_param_format_ = false;
  double origin_lat_        = 0.0;
  double origin_lon_        = 0.0;
  double origin_alt_        = 0.0;
  std::string origin_param_ = "";
  geometry_msgs::msg::TransformStamped earth_to_map;
  geographic_msgs::msg::GeoPoint::UniquePtr origin_;
  sensor_msgs::msg::NavSatFix::UniquePtr gps_pose_;
  std::smatch match_;
  bool using_ignition_tf_ = false;

public:
  Plugin() : as2_state_estimator_plugin_base::StateEstimatorBase(){};
  void on_setup() override {
    node_ptr_->get_parameter("use_gps", use_gps_);
    node_ptr_->get_parameter("set_origin_on_start", set_origin_on_start_);

    pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
        as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos,
        std::bind(&Plugin::pose_callback, this, std::placeholders::_1));
    twist_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::ground_truth::twist, as2_names::topics::ground_truth::qos,
        std::bind(&Plugin::twist_callback, this, std::placeholders::_1));

    // publish static transform from earth to map and map to odom
    earth_to_map = as2::tf::getTransformation(get_earth_frame(), get_map_frame(), 0, 0, 0, 0, 0, 0);
    if (!use_gps_) {
      // TODO: MODIFY this to a initial earth to map transform (reading initial position
      // from parameters or msgs )
      publish_static_transform(earth_to_map);
    } else {
      set_origin_srv_ = node_ptr_->create_service<as2_msgs::srv::SetOrigin>(
          as2_names::services::gps::set_origin,
          std::bind(&Plugin::setOriginCallback, this, std::placeholders::_1,
                    std::placeholders::_2));
      get_origin_srv_ = node_ptr_->create_service<as2_msgs::srv::GetOrigin>(
          as2_names::services::gps::get_origin,
          std::bind(&Plugin::getOriginCallback, this, std::placeholders::_1,
                    std::placeholders::_2));
      gps_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::NavSatFix>(
          as2_names::topics::sensor_measurements::gps, as2_names::topics::sensor_measurements::qos,
          std::bind(&Plugin::gps_callback, this, std::placeholders::_1));

      if (set_origin_on_start_) {
        node_ptr_->get_parameter("set_origin.lat", origin_lat_);
        node_ptr_->get_parameter("set_origin.lon", origin_lon_);
        node_ptr_->get_parameter("set_origin.alt", origin_alt_);

        origin_            = std::make_unique<geographic_msgs::msg::GeoPoint>();
        origin_->latitude  = origin_lat_;
        origin_->longitude = origin_lon_;
        origin_->altitude  = origin_alt_;

        RCLCPP_INFO(node_ptr_->get_logger(), "Origin set to %f, %f, %f", origin_lat_, origin_lon_,
                    origin_alt_);
      } else {
        RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for origin to be set");
      }
    }

    geometry_msgs::msg::TransformStamped map_to_odom =
        as2::tf::getTransformation(get_map_frame(), get_odom_frame(), 0, 0, 0, 0, 0, 0);

    // TODO: CHECK IF WE NEED TO PUBLISH THIS PERIODICALLY
    if (node_ptr_->has_parameter("use_ignition_tf")) {
      node_ptr_->get_parameter("use_ignition_tf", using_ignition_tf_);
      if (using_ignition_tf_) RCLCPP_INFO(node_ptr_->get_logger(), "Using ignition tfs");
    }
    publish_static_transform(earth_to_map);
    publish_static_transform(map_to_odom);
  };

private:
  void generate_map_frame_from_gps(const geographic_msgs::msg::GeoPoint &origin,
                                   const sensor_msgs::msg::NavSatFix &gps_pose) {
    as2::gps::GpsHandler gps_handler;
    gps_handler.setOrigin(origin.latitude, origin.longitude, origin.altitude);
    double x, y, z;
    gps_handler.LatLon2Local(gps_pose.latitude, gps_pose.longitude, gps_pose.altitude, x, y, z);
    earth_to_map = as2::tf::getTransformation(get_earth_frame(), get_map_frame(), x, y, z, 0, 0, 0);
    publish_static_transform(earth_to_map);
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // since it's ground_truth we consider that the frame obtained is the world frame (earth)
    // so we need to publish the transform from world to base_link, with map and odom as the
    // identity transform in order to keep the continuity of the tf tree we will modify the
    // odom->base_link transform

    if (msg->header.frame_id != get_earth_frame()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Received pose in frame %s, expected %s",
                   msg->header.frame_id.c_str(), get_earth_frame().c_str());
      return;
    }
    tf2::Transform transform;
    transform.setOrigin(
        tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    transform.setRotation(tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                                          msg->pose.orientation.z, msg->pose.orientation.w));
    // transform.child_frame_id          = get_base_frame();

    tf2::Transform odom_to_baselink;
    tf2::Transform earth_to_map_;

    tf2::fromMsg(earth_to_map.transform, earth_to_map_);

    convert_earth_to_baselink_2_odom_to_baselink_transform(transform, odom_to_baselink,
                                                           earth_to_map_);

    auto odom_to_baselink_msg            = geometry_msgs::msg::TransformStamped();
    odom_to_baselink_msg.header.stamp    = msg->header.stamp;
    odom_to_baselink_msg.header.frame_id = get_odom_frame();
    if (using_ignition_tf_) {
      odom_to_baselink_msg.child_frame_id = as2::tf::generateTfName("", node_ptr_->get_namespace());
    } else {
      odom_to_baselink_msg.child_frame_id = get_base_frame();
    }
    odom_to_baselink_msg.transform = tf2::toMsg(odom_to_baselink);

    publish_transform(odom_to_baselink_msg);

    // Pose should be published in the earth frame (world)
    // so we dont need to modify the frame_id
    publish_pose(*msg);
  };

  void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (msg->header.frame_id != get_base_frame()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Received twist in frame %s, expected %s",
                   msg->header.frame_id.c_str(), get_base_frame().c_str());
      // TODO: convert it to the base_link frame if needed
      return;
    }
    publish_twist(*msg);
  };

  void getOriginCallback(const as2_msgs::srv::GetOrigin::Request::SharedPtr request,
                         as2_msgs::srv::GetOrigin::Response::SharedPtr response) {
    if (origin_) {
      response->origin  = *origin_;
      response->success = true;
    } else {
      RCLCPP_WARN(node_ptr_->get_logger(), "Origin not set");
      response->success = false;
    }
  };

  void setOriginCallback(const as2_msgs::srv::SetOrigin::Request::SharedPtr request,
                         as2_msgs::srv::SetOrigin::Response::SharedPtr response) {
    if (origin_) {
      RCLCPP_WARN(node_ptr_->get_logger(), "Origin already set");
      response->success = false;
    } else {
      origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>(request->origin);
      RCLCPP_INFO(node_ptr_->get_logger(), "Origin set to %f, %f, %f", origin_->latitude,
                  origin_->longitude, origin_->altitude);
      response->success = true;
      generate_map_frame_from_gps(request->origin, *gps_pose_);
    }
  };

  void gps_callback(sensor_msgs::msg::NavSatFix::UniquePtr msg) {
    // This sould only be called when the use_gps_origin is true
    if (gps_pose_) {
      gps_sub_.reset();
      return;
    }
    gps_pose_ = std::move(msg);

    RCLCPP_INFO(node_ptr_->get_logger(), "GPS Callback: Map GPS pose set to %f, %f, %f",
                gps_pose_->latitude, gps_pose_->longitude, gps_pose_->altitude);

    generate_map_frame_from_gps(*origin_, *gps_pose_);
  };

};      // class GroundTruth
};      // namespace ground_truth
#endif  // __GROUND_TRUTH_HPP__
