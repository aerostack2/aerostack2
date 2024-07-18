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
* @file as2_external_object_to_tf.hpp
*
* as2_external_object_to_tf header file.
*
* @author Javilinos
*/

#ifndef AS2_EXTERNAL_OBJECT_TO_TF__AS2_EXTERNAL_OBJECT_TO_TF_HPP_
#define AS2_EXTERNAL_OBJECT_TO_TF__AS2_EXTERNAL_OBJECT_TO_TF_HPP_

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/utils.h>
#include <tf2_msgs/msg/tf_message.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <tuple>

#include "as2_msgs/srv/add_static_transform.hpp"
#include "as2_msgs/srv/add_static_transform_gps.hpp"
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/utils/gps_utils.hpp"

#include "as2_msgs/srv/get_origin.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

struct gps_object
{
  sensor_msgs::msg::NavSatFix::SharedPtr gps_pose;
  std_msgs::msg::Float32::SharedPtr azimuth;
};

class As2ExternalObjectToTf : public as2::Node
{
public:
  As2ExternalObjectToTf();

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  void setupNode();
  void cleanupNode();
  void run();

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  bool origin_set_ = false;
  bool use_sim_time = false;

  std::string config_path_;
  std::string mocap_topic_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr> gps_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> azimuth_subs_;
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr mocap_sub_;
  rclcpp::Service<as2_msgs::srv::AddStaticTransform>::SharedPtr setTrasformSrv;
  rclcpp::Service<as2_msgs::srv::AddStaticTransformGps>::SharedPtr
    setTrasformGpsSrv;
  geographic_msgs::msg::GeoPoint::UniquePtr origin_;

  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
  objects_subscriptions_;

  rclcpp::Client<as2_msgs::srv::GetOrigin>::SharedPtr get_origin_srv_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;
  std::unique_ptr<as2::gps::GpsHandler> gps_handler;
  std::map<std::string, gps_object> gps_poses;

  void loadObjects(const std::string path);

  void setupGPS();

  geometry_msgs::msg::TransformStamped gpsToTransform(
    const sensor_msgs::msg::NavSatFix::SharedPtr gps_pose,
    const float azimuth,
    const float pitch,
    const std::string frame_id,
    const std::string parent_frame_id);

  geometry_msgs::msg::Quaternion azimuthToQuaternion(
    const std_msgs::msg::Float32::SharedPtr azimuth);

  geometry_msgs::msg::Quaternion azimuthPitchToQuaternion(const float azimuth, const float pitch);

  void publishPoseAsTransform(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    std::string frame_id,
    std::string parent_frame_id);

  void publishPoseAsTransform(
    const geometry_msgs::msg::Pose::SharedPtr msg,
    std::string frame_id,
    std::string parent_frame_id);

  void poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    std::string frame_id,
    std::string parent_frame_id);
  void gpsCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg,
    std::string frame_id,
    std::string parent_frame_id);
  void azimuthCallback(
    const std_msgs::msg::Float32::SharedPtr msg,
    std::string frame_id,
    std::string parent_frame_id);

  void mocapCallback(
    const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg,
    std::vector<std::tuple<std::string, std::string>> mappings);

  void addStaticTransform(
    const as2_msgs::srv::AddStaticTransform::Request::SharedPtr request,
    const as2_msgs::srv::AddStaticTransform::Response::SharedPtr response);

  void addStaticTransformGps(
    const as2_msgs::srv::AddStaticTransformGps::Request::SharedPtr request,
    const as2_msgs::srv::AddStaticTransformGps::Response::SharedPtr response);
};
#endif  // AS2_EXTERNAL_OBJECT_TO_TF__AS2_EXTERNAL_OBJECT_TO_TF_HPP_
