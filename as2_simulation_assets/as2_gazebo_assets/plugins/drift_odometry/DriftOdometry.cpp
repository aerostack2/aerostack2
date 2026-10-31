// Copyright 2026 Universidad Politécnica de Madrid
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


#include "DriftOdometry.hpp"

#include <gz/msgs/odometry_with_covariance.pb.h>

#include <cstdint>
#include <memory>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <gz/transport/TopicUtils.hh>
#include "gz_compat.hpp"

namespace as2_gazebo_assets
{

namespace
{

template<typename T>
T param(const std::shared_ptr<const sdf::Element> & sdf, const std::string & name, T fallback)
{
  return sdf->Get<T>(name, fallback).first;
}

void setVector(const gz::math::Vector3d & v, gz::msgs::Vector3d * out)
{
  out->set_x(v.X());
  out->set_y(v.Y());
  out->set_z(v.Z());
}

void setCovariance(const Covariance6 & covariance, gz::msgs::Float_V * out)
{
  for (double value : covariance) {
    out->add_data(static_cast<float>(value));
  }
}

}  // namespace

void DriftOdometryPublisher::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager & /*event_manager*/)
{
  const gz::sim::Model model(entity);
  if (!model.Valid(ecm)) {
    gzerr << "DriftOdometryPublisher must be attached to a model." << std::endl;
    return;
  }
  const std::string model_name = model.Name(ecm);

  link_ = gz::sim::Link(model.CanonicalLink(ecm));
  if (!link_.Valid(ecm)) {
    gzerr << "DriftOdometryPublisher could not find the canonical link of model ["
          << model_name << "]." << std::endl;
    return;
  }
  // Ask the physics engine to fill in link velocities.
  link_.EnableVelocityChecks(ecm, true);

  DriftOdometryParams params;
  params.linear_velocity_noise =
    param(sdf, "linear_velocity_noise", params.linear_velocity_noise);
  params.linear_bias_noise = param(sdf, "linear_bias_noise", params.linear_bias_noise);
  params.yaw_rate_noise = param(sdf, "yaw_rate_noise", params.yaw_rate_noise);
  params.yaw_rate_bias_noise = param(sdf, "yaw_rate_bias_noise", params.yaw_rate_bias_noise);
  params.scale_error = param(sdf, "scale_error", params.scale_error);
  params.twist_linear_noise = param(sdf, "twist_linear_noise", params.twist_linear_noise);
  params.twist_angular_noise = param(sdf, "twist_angular_noise", params.twist_angular_noise);
  params.drift_horizontal = param(sdf, "drift_horizontal", params.drift_horizontal);
  params.drift_vertical = param(sdf, "drift_vertical", params.drift_vertical);
  params.drift_yaw = param(sdf, "drift_yaw", params.drift_yaw);
  const std::string drift_start = param<std::string>(sdf, "drift_start", "simulation");
  if (drift_start == "takeoff") {
    params.drift_start = DriftStart::kTakeoff;
  } else if (drift_start != "simulation") {
    gzwarn << "DriftOdometryPublisher: unknown drift_start [" << drift_start
           << "], expected simulation or takeoff. Using simulation." << std::endl;
  }
  params.takeoff_height = param(sdf, "takeoff_height", params.takeoff_height);
  const int seed = param(sdf, "seed", 0);
  drift_ = std::make_unique<DriftOdometryModel>(params, static_cast<std::uint64_t>(seed));

  // Names follow Gazebo's OdometryPublisher.
  odom_frame_id_ = param<std::string>(sdf, "odom_frame", model_name + "/odom");
  base_frame_id_ = param<std::string>(sdf, "robot_base_frame", model_name + "/base_link");

  const double publish_rate = param(sdf, "odom_publish_frequency", 100.0);
  if (publish_rate > 0.0) {
    publish_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / publish_rate));
  }

  const std::string topic = gz::transport::TopicUtils::AsValidTopic(
    param<std::string>(sdf, "odom_topic", "/model/" + model_name + "/odometry_drift"));
  if (topic.empty()) {
    gzerr << "DriftOdometryPublisher got an invalid topic." << std::endl;
    return;
  }
  publisher_ = node_.Advertise<gz::msgs::OdometryWithCovariance>(topic);
  gzmsg << "DriftOdometryPublisher publishing on [" << topic << "]" << std::endl;
}

void DriftOdometryPublisher::PostUpdate(
  const gz::sim::UpdateInfo & info,
  const gz::sim::EntityComponentManager & ecm)
{
  GZ_PROFILE("DriftOdometryPublisher::PostUpdate");
  if (info.paused || !drift_ || !publisher_.Valid()) {
    return;
  }

  const auto world_pose = link_.WorldPose(ecm);
  if (!world_pose) {
    return;
  }
  // Velocities are reported in the body frame. They are not available on
  // the first step, before physics has filled them in.
  Twist true_twist;
  const auto linear = link_.WorldLinearVelocity(ecm);
  const auto angular = link_.WorldAngularVelocity(ecm);
  if (linear && angular) {
    true_twist.linear = world_pose->Rot().RotateVectorReverse(*linear);
    true_twist.angular = world_pose->Rot().RotateVectorReverse(*angular);
  }

  // Propagate drift every step, so it does not depend on the publish rate.
  const double time = std::chrono::duration<double>(info.simTime).count();
  const Odometry odom = drift_->update(time, *world_pose, true_twist);

  // Publish on the same time grid as Gazebo's OdometryPublisher, so both
  // can be matched by stamp. A simulation reset moves time backwards; start
  // the grid again then.
  if (info.simTime < last_publish_time_) {
    last_publish_time_ = std::chrono::steady_clock::duration::zero();
  }
  if (info.simTime - last_publish_time_ < publish_period_) {
    return;
  }
  last_publish_time_ = info.simTime;

  gz::msgs::OdometryWithCovariance msg;
  auto * header = msg.mutable_header();
  header->mutable_stamp()->CopyFrom(gz::sim::convert<gz::msgs::Time>(info.simTime));
  auto * frame = header->add_data();
  frame->set_key("frame_id");
  frame->add_value(odom_frame_id_);
  auto * child_frame = header->add_data();
  child_frame->set_key("child_frame_id");
  child_frame->add_value(base_frame_id_);

  auto * pose = msg.mutable_pose_with_covariance();
  setVector(odom.pose.Pos(), pose->mutable_pose()->mutable_position());
  auto * orientation = pose->mutable_pose()->mutable_orientation();
  orientation->set_w(odom.pose.Rot().W());
  orientation->set_x(odom.pose.Rot().X());
  orientation->set_y(odom.pose.Rot().Y());
  orientation->set_z(odom.pose.Rot().Z());
  setCovariance(odom.pose_covariance, pose->mutable_covariance());

  auto * twist = msg.mutable_twist_with_covariance();
  setVector(odom.twist.linear, twist->mutable_twist()->mutable_linear());
  setVector(odom.twist.angular, twist->mutable_twist()->mutable_angular());
  setCovariance(odom.twist_covariance, twist->mutable_covariance());

  publisher_.Publish(msg);
}

}  // namespace as2_gazebo_assets

GZ_ADD_PLUGIN(
  as2_gazebo_assets::DriftOdometryPublisher,
  gz::sim::System,
  as2_gazebo_assets::DriftOdometryPublisher::ISystemConfigure,
  as2_gazebo_assets::DriftOdometryPublisher::ISystemPostUpdate)
