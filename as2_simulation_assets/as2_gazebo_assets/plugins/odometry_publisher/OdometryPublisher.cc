/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "OdometryPublisher.hh"

#include <gz/msgs/header.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/pose_v.pb.h>

#include <cmath>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Rand.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::OdometryPublisherPrivate
{
  /// \brief Calculates odometry and publishes an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
public:
  void UpdateOdometry(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm);

  /// \brief Gazebo communication node.
public:
  transport::Node node;

  /// \brief Model interface
public:
  Model model{kNullEntity};

  /// \brief Name of the world-fixed coordinate frame for the odometry message.
public:
  std::string odomFrame;

  /// \brief Name of the coordinate frame rigidly attached to the mobile
  /// robot base.
public:
  std::string robotBaseFrame;

  /// \brief Number of dimensions to represent odometry.
public:
  int dimensions{2};

  /// \brief Update period calculated from <odom_publish_frequency>.
public:
  std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
public:
  std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Odometry message publisher.
public:
  transport::Node::Publisher odomPub;

  /// \brief Odometry with covariance message publisher.
public:
  transport::Node::Publisher odomCovPub;

  /// \brief Pose vector (TF) message publisher.
public:
  transport::Node::Publisher tfPub;

  /// \brief Initialized flag.
public:
  bool initialized{false};

  /// \brief Current integrated pose in the odom frame.
public:
  math::Pose3d pose{0, 0, 0, 0, 0, 0};

  /// \brief Last update timestamp.
public:
  std::chrono::steady_clock::duration lastUpdateTime{0};

  /// \brief Allow specifying constant xyz and rpy offsets
public:
  math::Pose3d offset{0, 0, 0, 0, 0, 0};

  /// \brief Position noise standard deviation (meters)
public:
  double positionNoiseStdDev{0.0};

  /// \brief Angular noise standard deviation (radians)
public:
  double angularNoiseStdDev{0.0};
};

//////////////////////////////////////////////////
OdometryPublisher::OdometryPublisher()
: dataPtr(std::make_unique<OdometryPublisherPrivate>())
{
}

//////////////////////////////////////////////////
void OdometryPublisher::Configure(
  const Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  EntityComponentManager & _ecm,
  EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm)) {
    gzerr << "OdometryPublisher system plugin should be attached to a model"
          << " entity. Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->odomFrame = this->dataPtr->model.Name(_ecm) + "/" + "odom";
  if (!_sdf->HasElement("odom_frame")) {
    gzdbg << "OdometryPublisher system plugin missing <odom_frame>, "
          << "defaults to \"" << this->dataPtr->odomFrame << "\"" << std::endl;
  } else {
    this->dataPtr->odomFrame = _sdf->Get<std::string>("odom_frame");
  }

  if (_sdf->HasElement("xyz_offset")) {
    this->dataPtr->offset.Pos() = _sdf->Get<math::Vector3d>("xyz_offset");
  }

  if (_sdf->HasElement("rpy_offset")) {
    this->dataPtr->offset.Rot() =
      math::Quaterniond(_sdf->Get<math::Vector3d>("rpy_offset"));
  }

  // Read noise parameters (separate for position and orientation)
  if (_sdf->HasElement("gaussian_noise")) {
    // Legacy support: use same noise for both position and angular
    double noise = _sdf->Get<double>("gaussian_noise");
    this->dataPtr->positionNoiseStdDev = noise;
    this->dataPtr->angularNoiseStdDev = noise;
  }

  if (_sdf->HasElement("position_noise")) {
    this->dataPtr->positionNoiseStdDev = _sdf->Get<double>("position_noise");
  }

  if (_sdf->HasElement("angular_noise")) {
    this->dataPtr->angularNoiseStdDev = _sdf->Get<double>("angular_noise");
  }

  this->dataPtr->robotBaseFrame = this->dataPtr->model.Name(_ecm) +
    "/" + "base_footprint";
  if (!_sdf->HasElement("robot_base_frame")) {
    gzdbg << "OdometryPublisher system plugin missing <robot_base_frame>, "
          << "defaults to \"" << this->dataPtr->robotBaseFrame << "\"" << std::endl;
  } else {
    this->dataPtr->robotBaseFrame = _sdf->Get<std::string>("robot_base_frame");
  }

  this->dataPtr->dimensions = 2;
  if (!_sdf->HasElement("dimensions")) {
    gzdbg << "OdometryPublisher system plugin missing <dimensions>, "
          << "defaults to \"" << this->dataPtr->dimensions << "\"" << std::endl;
  } else {
    this->dataPtr->dimensions = _sdf->Get<int>("dimensions");
    if (this->dataPtr->dimensions != 2 && this->dataPtr->dimensions != 3) {
      gzerr << "OdometryPublisher system plugin <dimensions> must be 2D or 3D "
            << "not " << this->dataPtr->dimensions
            << "D. Failed to initialize." << std::endl;
      return;
    }
  }

  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  if (odomFreq > 0) {
    std::chrono::duration<double> period{1 / odomFreq};
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }

  // Setup odometry
  std::string odomTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/odometry"};
  std::string odomCovTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/odometry_with_covariance"};
  if (_sdf->HasElement("odom_topic")) {
    odomTopic = _sdf->Get<std::string>("odom_topic");
  }
  if (_sdf->HasElement("odom_covariance_topic")) {
    odomCovTopic = _sdf->Get<std::string>("odom_covariance_topic");
  }

  std::string odomTopicValid {transport::TopicUtils::AsValidTopic(odomTopic)};
  if (odomTopicValid.empty()) {
    gzerr << "Failed to generate odom topic ["
          << odomTopic << "]" << std::endl;
  } else {
    this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopicValid);
    gzmsg << "OdometryPublisher publishing odometry on [" << odomTopicValid
          << "]" << std::endl;
  }

  std::string odomCovTopicValid {
    transport::TopicUtils::AsValidTopic(odomCovTopic)};
  if (odomCovTopicValid.empty()) {
    gzerr << "Failed to generate odom topic ["
          << odomCovTopic << "]" << std::endl;
  } else {
    this->dataPtr->odomCovPub = this->dataPtr->node.Advertise<
      msgs::OdometryWithCovariance>(odomCovTopicValid);
    gzmsg << "OdometryPublisher publishing odometry with covariance on ["
          << odomCovTopicValid << "]" << std::endl;
  }

  std::string tfTopic{"/model/" + this->dataPtr->model.Name(_ecm) + "/pose"};
  if (_sdf->HasElement("tf_topic")) {
    tfTopic = _sdf->Get<std::string>("tf_topic");
  }
  std::string tfTopicValid {transport::TopicUtils::AsValidTopic(tfTopic)};
  if (tfTopicValid.empty()) {
    gzerr << "Failed to generate valid TF topic from [" << tfTopic << "]"
          << std::endl;
  } else {
    this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
      tfTopicValid);
    gzmsg << "OdometryPublisher publishing Pose_V (TF) on ["
          << tfTopicValid << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void OdometryPublisher::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("OdometryPublisher::PreUpdate");

  if (kNullEntity == this->dataPtr->model.Entity()) {
    return;
  }

  if (!this->dataPtr->model.Valid(_ecm)) {
    gzwarn << "OdometryPublisher model no longer valid. "
           << "Disabling plugin." << std::endl;
    this->dataPtr->model = Model(kNullEntity);
    return;
  }

  // Support rewind warning
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }
}

//////////////////////////////////////////////////
void OdometryPublisher::PostUpdate(
  const UpdateInfo & _info,
  const EntityComponentManager & _ecm)
{
  GZ_PROFILE("OdometryPublisher::PostUpdate");

  if (kNullEntity == this->dataPtr->model.Entity()) {
    return;
  }

  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void OdometryPublisherPrivate::UpdateOdometry(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("OdometryPublisher::UpdateOdometry");

  // Initialize on first call
  if (!this->initialized) {
    // Initialize pose from world pose with offset
    const math::Pose3d rawPose = worldPose(this->model.Entity(), _ecm);
    this->pose = rawPose * this->offset;
    this->lastUpdateTime = _info.simTime;
    this->initialized = true;
    return;
  }

  // Throttle publishing
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->odomPubPeriod) {
    return;
  }

  const std::chrono::duration<double> dt =
    std::chrono::duration<double>(_info.simTime - this->lastUpdateTime);

  // Do not update if the time interval is zero (or near zero).
  if (math::equal(0.0, dt.count()) || dt.count() < 1e-9) {
    return;
  }

  auto linearVelComp = _ecm.Component<components::LinearVelocity>(
    this->model.Entity());
  auto angularVelComp = _ecm.Component<components::AngularVelocity>(
    this->model.Entity());

  if (!linearVelComp || !angularVelComp) {
    // Components not available yet, skip this update
    return;
  }

  math::Vector3d linVel = linearVelComp->Data();
  math::Vector3d angVel = angularVelComp->Data();

  // Add velocity noise using random walk model (noise scales with 1/sqrt(dt))
  double velNoise = this->positionNoiseStdDev / std::sqrt(dt.count());
  double angNoise = this->angularNoiseStdDev / std::sqrt(dt.count());

  math::Vector3d velBody(
    linVel.X() + math::Rand::DblNormal(0.0, velNoise),
    linVel.Y() + math::Rand::DblNormal(0.0, velNoise),
    (this->dimensions == 3) ? linVel.Z() + math::Rand::DblNormal(0.0, velNoise) : 0.0
  );

  math::Vector3d angVel(
    (this->dimensions == 3) ? angVel.X() + math::Rand::DblNormal(0.0, angNoise) : 0.0,
    (this->dimensions == 3) ? angVel.Y() + math::Rand::DblNormal(0.0, angNoise) : 0.0,
    angVel.Z() + math::Rand::DblNormal(0.0, angNoise)
  );

  // Integrate orientation using quaternion from body-frame angular velocity
  double omegaMag = angVel.Length();
  math::Quaterniond q_delta = math::Quaterniond::Identity;

  // Exponential map integration: https://math.stackexchange.com/a/831788
  if (omegaMag > 1e-10) {
    math::Vector3d axis = angVel / omegaMag;
    double angle = omegaMag * dt.count();
    q_delta.SetFromAxisAngle(axis, angle);
  }

  math::Quaterniond qNew = this->pose.Rot() * q_delta;
  qNew.Normalize();

  // Integrate position (transform body velocity to world frame first)
  math::Vector3d velWorld = this->pose.Rot().RotateVector(velBody);
  this->pose.Pos() += velWorld * dt.count();
  this->pose.Rot() = qNew;

  // Construct the odometry message
  msgs::Odometry msg;

  // Set timestamp
  msgs::Header header;
  header.mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));

  // Set frame IDs
  auto frame = header.add_data();
  frame->set_key("frame_id");
  frame->add_value(odomFrame);
  auto childFrame = header.add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(robotBaseFrame);

  msg.mutable_header()->CopyFrom(header);

  // Set pose (in odom frame)
  msg.mutable_pose()->mutable_position()->set_x(this->pose.Pos().X());
  msg.mutable_pose()->mutable_position()->set_y(this->pose.Pos().Y());
  if (this->dimensions == 3) {
    msg.mutable_pose()->mutable_position()->set_z(this->pose.Pos().Z());
  }

  msgs::Set(msg.mutable_pose()->mutable_orientation(), this->pose.Rot());

  msg.mutable_twist()->mutable_linear()->set_x(velBody.X());
  msg.mutable_twist()->mutable_linear()->set_y(velBody.Y());
  msg.mutable_twist()->mutable_linear()->set_z(velBody.Z());

  msg.mutable_twist()->mutable_angular()->set_x(angVel.X());
  msg.mutable_twist()->mutable_angular()->set_y(angVel.Y());
  msg.mutable_twist()->mutable_angular()->set_z(angVel.Z());

  // Update time tracking
  this->lastUpdateTime = _info.simTime;
  this->lastOdomPubTime = _info.simTime;

  // Publish odometry message
  if (this->odomPub.Valid()) {
    this->odomPub.Publish(msg);
  }

  // Generate and publish odometry with covariance message
  msgs::OdometryWithCovariance msgCovariance;

  // Set the time stamp in the header
  msgCovariance.mutable_header()->CopyFrom(header);

  // Copy pose from odometry msg
  msgCovariance.mutable_pose_with_covariance()->
    mutable_pose()->mutable_position()->set_x(msg.pose().position().x());
  msgCovariance.mutable_pose_with_covariance()->
    mutable_pose()->mutable_position()->set_y(msg.pose().position().y());
  msgCovariance.mutable_pose_with_covariance()->
    mutable_pose()->mutable_position()->set_z(msg.pose().position().z());

  // Copy orientation from odometry msg
  msgs::Set(
    msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
    mutable_orientation(), this->pose.Rot());

  // Copy twist from odometry msg
  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_angular()->set_x(msg.twist().angular().x());
  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_angular()->set_y(msg.twist().angular().y());
  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_angular()->set_z(msg.twist().angular().z());

  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_linear()->set_x(msg.twist().linear().x());
  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_linear()->set_y(msg.twist().linear().y());
  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_linear()->set_z(msg.twist().linear().z());

  // Populate covariance matrices (6x6, row-major order)
  // Position covariance (m²)
  double posVariance = this->positionNoiseStdDev * this->positionNoiseStdDev;
  // Orientation covariance (rad²)
  double angVariance = this->angularNoiseStdDev * this->angularNoiseStdDev;

  // Velocity variances (from noise propagation through derivative)
  double velVariance = (velNoise * velNoise);
  double angVelVariance = (angNoise * angNoise);

  // Pose covariance (diagonal elements only)
  for (int i = 0; i < 36; i++) {
    if (i % 7 == 0) {  // Diagonal elements (0, 7, 14, 21, 28, 35)
      int idx = i / 7;
      double value = 0.0;
      if (idx < 3) {
        value = posVariance;  // x, y, z
      } else {
        value = angVariance;  // roll, pitch, yaw
      }
      msgCovariance.mutable_pose_with_covariance()->
        mutable_covariance()->add_data(value);
    } else {
      msgCovariance.mutable_pose_with_covariance()->
        mutable_covariance()->add_data(0.0);
    }
  }

  // Twist covariance (diagonal elements only)
  for (int i = 0; i < 36; i++) {
    if (i % 7 == 0) {  // Diagonal elements
      int idx = i / 7;
      double value = 0.0;
      if (idx < 3) {
        value = velVariance;  // vx, vy, vz
      } else {
        value = angVelVariance;  // wx, wy, wz
      }
      msgCovariance.mutable_twist_with_covariance()->
        mutable_covariance()->add_data(value);
    } else {
      msgCovariance.mutable_twist_with_covariance()->
        mutable_covariance()->add_data(0.0);
    }
  }

  // Publish covariance message
  if (this->odomCovPub.Valid()) {
    this->odomCovPub.Publish(msgCovariance);
  }

  // Publish TF message
  if (this->tfPub.Valid()) {
    msgs::Pose_V tfMsg;
    auto tfMsgPose = tfMsg.add_pose();
    tfMsgPose->CopyFrom(msg.pose());
    tfMsgPose->mutable_header()->CopyFrom(header);

    this->tfPub.Publish(tfMsg);
  }
}

GZ_ADD_PLUGIN(
  OdometryPublisher,
  gz::sim::System,
  OdometryPublisher::ISystemConfigure,
  OdometryPublisher::ISystemPreUpdate,
  OdometryPublisher::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  OdometryPublisher,
  "gz::sim::systems::OdometryPublisher")
