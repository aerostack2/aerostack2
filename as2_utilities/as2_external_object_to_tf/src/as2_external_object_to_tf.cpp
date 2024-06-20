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

/*!*******************************************************************************************
 *  \file       as2_external_object_to_tf.cpp
 *  \brief      External Object to TF source file.
 *  \authors    Javier Melero Deza
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#include "as2_external_object_to_tf.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "yaml-cpp/yaml.h"

As2ExternalObjectToTf::As2ExternalObjectToTf()
: as2::Node("external_object_to_tf")
{
  try {
    this->declare_parameter("config_file", "config/external_objects.yaml");
    this->get_parameter("config_file", config_path_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "config_file parameter not set: %s", e.what());
    config_path_ = "config/external_objects.yaml";
  }

  try {
    this->declare_parameter("mocap_topic", "mocap/pose");
    this->get_parameter("mocap_topic", mocap_topic_);
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "mocap_topic parameter not set: %s", e.what());
    mocap_topic_ = "mocap/pose";
  }
  this->get_parameter("use_sim_time", use_sim_time);
}

void As2ExternalObjectToTf::poseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr _msg,
  std::string frame_id,
  std::string parent_frame_id)
{
  if (parent_frame_id == "msg_defined") {
    parent_frame_id = _msg->header.frame_id;
  }
  publishPoseAsTransform(_msg, frame_id, parent_frame_id);
}

void As2ExternalObjectToTf::gpsCallback(
  const sensor_msgs::msg::NavSatFix::SharedPtr _msg,
  std::string frame_id,
  std::string parent_frame_id)
{
  gps_poses[frame_id].gps_pose = _msg;

  if (gps_poses[frame_id].azimuth != NULL) {
    tfBroadcaster->sendTransform(
      gpsToTransform(
        gps_poses[frame_id].gps_pose,
        gps_poses[frame_id].azimuth->data, 0.0, frame_id,
        parent_frame_id));
  }
}

void As2ExternalObjectToTf::azimuthCallback(
  const std_msgs::msg::Float32::SharedPtr _msg,
  std::string frame_id,
  std::string parent_frame_id)
{
  gps_poses[frame_id].azimuth = _msg;
}

void As2ExternalObjectToTf::mocapCallback(
  const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg,
  std::vector<std::tuple<std::string, std::string>> mappings)
{
  for (auto mapping : mappings) {
    for (auto body : msg->rigidbodies) {
      if (body.rigid_body_name == std::get<0>(mapping)) {
        std::shared_ptr<geometry_msgs::msg::Pose> pose =
          std::make_shared<geometry_msgs::msg::Pose>(body.pose);
        publishPoseAsTransform(pose, std::get<1>(mapping), "earth");
      }
    }
  }
}

void As2ExternalObjectToTf::addStaticTransform(
  const std::shared_ptr<as2_external_object_to_tf::srv::AddStaticTransform::Request> request,
  const std::shared_ptr<as2_external_object_to_tf::srv::AddStaticTransform::Response> response)
{
  geometry_msgs::msg::TransformStamped static_transform;
  static_transform.child_frame_id = request->child_frame_id;
  static_transform.header.frame_id = request->frame_id;
  static_transform.header.stamp = this->get_clock()->now();
  static_transform.transform = request->transform;
  staticTfBroadcaster->sendTransform(static_transform);
  response->success = true;
}

void As2ExternalObjectToTf::addStaticTransformGps(
  const std::shared_ptr<as2_external_object_to_tf::srv::AddStaticTransformGps::Request> request,
  const std::shared_ptr<as2_external_object_to_tf::srv::AddStaticTransformGps::Response>
  response)
{
  if (!origin_set_) {
    setupGPS();
    origin_set_ = true;
  }
  geometry_msgs::msg::TransformStamped static_transform;
  static_transform = gpsToTransform(
    std::make_shared<sensor_msgs::msg::NavSatFix>(request->gps_position), request->azimuth,
    request->elevation, request->child_frame_id, request->frame_id);
  static_transform.header.stamp = this->get_clock()->now();
  staticTfBroadcaster->sendTransform(static_transform);
  response->success = true;
}

geometry_msgs::msg::Quaternion As2ExternalObjectToTf::azimuthToQuaternion(
  std_msgs::msg::Float32::SharedPtr azimuth)
{
  float azimuthRad = azimuth->data * M_PI / 180.0;
  azimuthRad += M_PI / 2;
  if (azimuthRad > M_PI) {
    azimuthRad -= (2 * M_PI);
  }
  geometry_msgs::msg::Quaternion q;
  double halfYaw = azimuthRad * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = cos(halfYaw);
  q.w = sin(halfYaw);
  return q;
}

geometry_msgs::msg::Quaternion As2ExternalObjectToTf::azimuthPitchToQuaternion(
  float azimuth = 0.0,
  float elevation = 0.0)
{
  double rotated_azimuth = azimuth - 90;
  if (rotated_azimuth < 0) {
    rotated_azimuth += 360.0;
  } else if (rotated_azimuth >= 360.0) {
    rotated_azimuth -= 360.0;
  }
  float azimuthRad = rotated_azimuth * M_PI / 180.0;

  float elevationRad = elevation * M_PI / 180.0;
  if (elevationRad > M_PI) {
    elevationRad -= 2.0 * M_PI;
  } else if (elevationRad < -M_PI) {
    elevationRad += 2.0 * M_PI;
  }

  geometry_msgs::msg::Quaternion q;
  as2::frame::eulerToQuaternion(0.0, elevationRad, -azimuthRad, q);
  return q;
}

geometry_msgs::msg::TransformStamped As2ExternalObjectToTf::gpsToTransform(
  const sensor_msgs::msg::NavSatFix::SharedPtr gps_pose,
  const float azimuth,
  const float elevation,
  const std::string frame_id,
  const std::string parent_frame_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = gps_pose->header.stamp;
  transform.header.frame_id = parent_frame_id;
  transform.child_frame_id = frame_id;
  sensor_msgs::msg::NavSatFix * rawPtr = gps_pose.get();
  gps_handler->LatLon2Local(
    *rawPtr, transform.transform.translation.x,
    transform.transform.translation.y, transform.transform.translation.z);
  transform.transform.rotation = azimuthPitchToQuaternion(azimuth, elevation);
  return transform;
}

void As2ExternalObjectToTf::publishPoseAsTransform(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg,
  std::string frame_id,
  std::string parent_frame_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.child_frame_id = frame_id;
  transform.header.frame_id = parent_frame_id;
  transform.header.stamp = msg->header.stamp;
  transform.transform.rotation = msg->pose.orientation;
  transform.transform.translation.x = msg->pose.position.x;
  transform.transform.translation.y = msg->pose.position.y;
  transform.transform.translation.z = msg->pose.position.z;
  tfBroadcaster->sendTransform(transform);
}

void As2ExternalObjectToTf::publishPoseAsTransform(
  const geometry_msgs::msg::Pose::SharedPtr msg,
  std::string frame_id,
  std::string parent_frame_id)
{
  geometry_msgs::msg::PoseStamped::SharedPtr pose_msg =
    std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose_msg->pose = *msg;
  pose_msg->header.stamp = this->get_clock()->now();
  publishPoseAsTransform(pose_msg, frame_id, parent_frame_id);
}

void As2ExternalObjectToTf::loadObjects(const std::string path)
{
  try {  // Load objects from yaml file
    RCLCPP_INFO(this->get_logger(), "Loading objects from config file: %s", config_path_.c_str());
    YAML::Node config = YAML::LoadFile(config_path_);  // Load the config file

    auto objects = config["objects"];  // Get the objects from the config file
    for (YAML::const_iterator object = objects.begin(); object != objects.end();
      ++object)                                                // Iterate over the objects
    {
      std::string type = (*object)["type"].as<std::string>();  // Get the type of the object

      if ((*object)["type"].as<std::string>() == "pose") {  // If the object is a pose
        std::string parent_frame = ((*object)["parent_frame"].IsDefined()) ?
          (*object)["parent_frame"].as<std::string>() :
          "msg_defined";

        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> poseFnc =
          std::bind(
          &As2ExternalObjectToTf::poseCallback, this, std::placeholders::_1,
          (*object)["frame"].as<std::string>(), parent_frame);

        pose_subs_.push_back(
          this->create_subscription<geometry_msgs::msg::PoseStamped>(
            (*object)["pose_topic"].as<std::string>(), as2_names::topics::self_localization::qos,
            poseFnc));

      } else if ((*object)["type"].as<std::string>() == "gps") {  // If the object is a gps
        if (!origin_set_) {
          setupGPS();
          origin_set_ = true;
        }

        As2ExternalObjectToTf::gps_poses[(*object)["frame"].as<std::string>()] = gps_object();
        std::string parent_frame = ((*object)["parent_frame"].IsDefined()) ?
          (*object)["parent_frame"].as<std::string>() :
          "earth";

        std::function<void(std::shared_ptr<sensor_msgs::msg::NavSatFix>)> gpsFnc =
          std::bind(
          &As2ExternalObjectToTf::gpsCallback, this, std::placeholders::_1,
          (*object)["frame"].as<std::string>(), parent_frame);

        std::function<void(std::shared_ptr<std_msgs::msg::Float32>)> azimuthFnc =
          std::bind(
          &As2ExternalObjectToTf::azimuthCallback, this, std::placeholders::_1,
          (*object)["frame"].as<std::string>(), parent_frame);

        gps_subs_.push_back(
          this->create_subscription<sensor_msgs::msg::NavSatFix>(
            (*object)["gps_topic"].as<std::string>(), as2_names::topics::self_localization::qos,
            gpsFnc));

        azimuth_subs_.push_back(
          this->create_subscription<std_msgs::msg::Float32>(
            (*object)["azimuth_topic"].as<std::string>(), as2_names::topics::self_localization::qos,
            azimuthFnc));

      } else if ((*object)["type"].as<std::string>() == "pose_static") {
        std::string parent_frame = ((*object)["parent_frame"].IsDefined()) ?
          (*object)["parent_frame"].as<std::string>() :
          "earth";
        std::string frame = (*object)["frame"].as<std::string>();
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.frame_id = parent_frame;
        static_transform.child_frame_id = frame;
        static_transform.transform.translation.x = (*object)["pose"]["position"]["x"].as<double>();
        static_transform.transform.translation.y = (*object)["pose"]["position"]["y"].as<double>();
        static_transform.transform.translation.z = (*object)["pose"]["position"]["z"].as<double>();
        double roll = (*object)["pose"]["orientation"]["r"].as<double>();
        double pitch = (*object)["pose"]["orientation"]["p"].as<double>();
        double yaw = (*object)["pose"]["orientation"]["y"].as<double>();
        geometry_msgs::msg::Quaternion quat;
        as2::frame::eulerToQuaternion(roll, pitch, yaw, quat);
        static_transform.transform.rotation = quat;
        staticTfBroadcaster->sendTransform(static_transform);

      } else if ((*object)["type"].as<std::string>() == "gps_static") {
        if (!origin_set_) {
          setupGPS();
          origin_set_ = true;
        }
        std::string parent_frame = ((*object)["parent_frame"].IsDefined()) ?
          (*object)["parent_frame"].as<std::string>() :
          "earth";
        std::string frame = (*object)["frame"].as<std::string>();
        std::shared_ptr<sensor_msgs::msg::NavSatFix> gps_pose =
          std::make_shared<sensor_msgs::msg::NavSatFix>();
        gps_pose->latitude = (*object)["gps_pose"]["lat"].as<double>();
        gps_pose->longitude = (*object)["gps_pose"]["lon"].as<double>();
        gps_pose->altitude = (*object)["gps_pose"]["alt"].as<double>();
        float bank = ((*object)["orientation"]["bank"].IsDefined()) ?
          (*object)["orientation"]["bank"].as<float>() :
          0.0;
        float elevation = ((*object)["orientation"]["elevation"].IsDefined()) ?
          (*object)["orientation"]["elevation"].as<float>() :
          0.0;
        float azimuth = ((*object)["orientation"]["azimuth"].IsDefined()) ?
          (*object)["orientation"]["azimuth"].as<float>() :
          0.0;
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform = gpsToTransform(gps_pose, azimuth, elevation, frame, parent_frame);
        staticTfBroadcaster->sendTransform(static_transform);

      } else if ((*object)["type"].as<std::string>() == "mocap") {
        std::vector<std::tuple<std::string, std::string>> mappings;
        for (auto mapping : (*object)["rigid_bodies"]) {
          mappings.push_back(
            std::make_tuple(
              mapping["rigid_body"].as<std::string>(),
              mapping["frame"].as<std::string>()));
        }
        std::function<void(mocap4r2_msgs::msg::RigidBodies::SharedPtr)> mocapFnc =
          std::bind(&As2ExternalObjectToTf::mocapCallback, this, std::placeholders::_1, mappings);

        this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
          mocap_topic_, as2_names::topics::self_localization::qos, mocapFnc);

      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Invalid type for object '%s', types are: pose || gps || pose_static || "
          "gps_static || mocap",
          type.c_str());
      }
      RCLCPP_INFO(
        this->get_logger(), "Object '%s' Succesfully loaded from config file",
        type.c_str());
    }
  } catch (YAML::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "YAML error: %s", e.what());
  }

  setTrasformSrv = this->create_service<as2_external_object_to_tf::srv::AddStaticTransform>(
    this->generate_local_name("add_static_transform"),
    std::bind(
      &As2ExternalObjectToTf::addStaticTransform, this, std::placeholders::_1,
      std::placeholders::_2));

  setTrasformGpsSrv = this->create_service<as2_external_object_to_tf::srv::AddStaticTransformGps>(
    this->generate_local_name("add_static_transform_gps"),
    std::bind(
      &As2ExternalObjectToTf::addStaticTransformGps, this, std::placeholders::_1,
      std::placeholders::_2));
}

void As2ExternalObjectToTf::setupNode()
{
  tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  staticTfBroadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  loadObjects(config_path_);
}

void As2ExternalObjectToTf::setupGPS()
{
  get_origin_srv_ = this->create_client<as2_msgs::srv::GetOrigin>(
    as2_names::services::gps::get_origin);    // Should be same origin for every drone ?

  while (!get_origin_srv_->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      break;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<as2_msgs::srv::GetOrigin::Request>();

  request->structure_needs_at_least_one_member = 0;

  bool success = false;  // TO-DO: Improve this
  // Wait for the result.
  while (!success) {
    auto result = get_origin_srv_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), result,
        std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // ;

    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service get origin");
      return;
    }
    auto result_obj = *result.get();
    success = result_obj.success;
    if (success) {
      origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>(result_obj.origin);
      RCLCPP_INFO(
        this->get_logger(), "Origin in: lat: %f, lon %f, alt: %f", origin_->latitude,
        origin_->longitude, origin_->altitude);
      gps_handler = std::make_unique<as2::gps::GpsHandler>(
        origin_->latitude, origin_->longitude,
        origin_->altitude);
    } else {
      RCLCPP_WARN(this->get_logger(), "Get origin request not successful, trying again...");
    }
  }
}

void As2ExternalObjectToTf::run() {}

void As2ExternalObjectToTf::cleanupNode() {}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn As2ExternalObjectToTf::on_configure(const rclcpp_lifecycle::State & _state)
{
  // Set subscriptions, publishers, services, actions, etc. here.
  setupNode();

  return CallbackReturn::SUCCESS;
}

CallbackReturn As2ExternalObjectToTf::on_deactivate(const rclcpp_lifecycle::State & _state)
{
  // Clean up subscriptions, publishers, services, actions, etc. here.
  return CallbackReturn::SUCCESS;
}

CallbackReturn As2ExternalObjectToTf::on_shutdown(const rclcpp_lifecycle::State & _state)
{
  // Clean other resources here.

  return CallbackReturn::SUCCESS;
}
