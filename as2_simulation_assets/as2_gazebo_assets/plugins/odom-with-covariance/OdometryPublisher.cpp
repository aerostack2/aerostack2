// Copyright 2025 Universidad Politécnica de Madrid
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

 #include "OdometryPublisher.hpp"

 #include <gz/msgs/header.pb.h>
 #include <gz/msgs/odometry.pb.h>
 #include <gz/msgs/odometry_with_covariance.pb.h>
 #include <gz/msgs/pose_v.pb.h>

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
 #include <gz/math/RollingMean.hh>
 #include <gz/plugin/Register.hh>
 #include <gz/transport/Node.hh>

 #include "gz/sim/components/Pose.hh"
 #include "gz/sim/components/JointPosition.hh"
 #include "gz/sim/Model.hh"
 #include "gz/sim/Util.hh"

using namespace gz;  // NOLINT
using namespace sim;  // NOLINT
using namespace systems;  // NOLINT

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
  //! [modelDeclaration]

public:
  Model model{kNullEntity};
  //! [modelDeclaration]

  /// \brief Name of the world-fixed coordinate frame for the odometry message.

public:
  std::string odomFrame;

  /// \brief Name of the coordinate frame rigidly attached to the mobile
  /// robot base.

public:
  std::string robotBaseFrame;

  /// \brief Number of dimensions to represent odometry.

public:
  int dimensions;

  /// \brief Update period calculated from <odom__publish_frequency>.

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
  ///////////// Añadida
  transport::Node::Publisher odomPubmod;

  /// \brief Odometry with covariance message publisher.

public:
  ///////////////// Añadida
  transport::Node::Publisher odomCovPubmod;

  /// \brief Pose vector (TF) message publisher.

public:
  transport::Node::Publisher tfPub;

  /// \brief Rolling mean accumulators for the linear velocity

public:
  std::tuple<math::RollingMean, math::RollingMean, math::RollingMean>
  linearMean;

  /// \brief Rolling mean accumulators for the angular velocity

public:
  std::tuple<math::RollingMean, math::RollingMean, math::RollingMean>
  angularMean;

  /// \brief Initialized flag.

public:
  bool initialized{false};

public:
  math::Pose3d initialPose;

  /// \brief Current pose of the model in the odom frame.

public:
  math::Pose3d lastUpdatePose{0, 0, 0, 0, 0, 0};

  /// \brief Current timestamp.

public:
  math::Pose3d lastUpdatePoseOdom{0, 0, 0, 0, 0, 0};

  /// \brief Current timestamp.

public:
  std::chrono::steady_clock::time_point lastUpdateTime;

  /// \brief Allow specifying constant xyz and rpy offsets

public:
  gz::math::Pose3d offset = {0, 0, 0, 0, 0, 0};

  /// \brief Gaussian noise

public:
  double gaussianNoise = 0.0;
  // Variables acumuladoras para el cálculo incremental de la varianza
  // (declarar como miembros de la clase)
  double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  double sumX2 = 0.0, sumY2 = 0.0, sumZ2 = 0.0;
  double sumRoll = 0.0, sumPitch = 0.0, sumYaw = 0.0;
  double sumRoll2 = 0.0, sumPitch2 = 0.0, sumYaw2 = 0.0;
  int sampleCount = 0;

public:
  double wrapAngle0To2Pi(const double theta)
  {
    double theta_wrapped = fmod(theta, 2.0 * M_PI);
    if (theta_wrapped < 0.0) {
      theta_wrapped += 2.0 * M_PI;
    }
    return theta_wrapped;
  }

  double angleMinError(const double theta1, const double theta2)
  {
    double theta1_wrapped = wrapAngle0To2Pi(theta1);
    double theta2_wrapped = wrapAngle0To2Pi(theta2);
    double error = theta1_wrapped - theta2_wrapped;
    if (error > M_PI) {
      error -= 2.0 * M_PI;
    } else if (error < -M_PI) {
      error += 2.0 * M_PI;
    }
    return error;
  }

  /**
 * @brief Compute the quaternion derivative.
 *
 * This function computes the quaternion derivative using Hamiltonian product:
 * q_dot = 0.5 * q * omega_q
 *
 * @param q The input quaternion
 * @param omega The angular velocity vector used for integration
 * @return math::Quaterniond The quaternion derivative
 */
  math::Quaterniond get_quaternion_derivative(
    const math::Quaterniond & q,
    const math::Vector3d & omega)
  {
    // Convert the angular velocity vector to a pure quaternion (w=0)
    math::Quaterniond omega_q(0, omega.X(), omega.Y(), omega.Z());

    // Compute quaternion derivative using Hamiltonian product
    math::Quaterniond q_dot = (q * omega_q) * 0.5;

    return q_dot;
  }

/**
 * @brief Integrate a quaternion
 *
 * This function integrates a quaternion using Euler integration:
 * q_{k+1} = q_k + q_dot * dt
 *
 * @param q The input quaternion
 * @param omega The angular velocity vector used for integration
 * @param dt The integration time step
 * @return math::Quaterniond The integrated quaternion
 */
  math::Quaterniond get_quaternion_integrate(
    const math::Quaterniond & q,
    const math::Vector3d & omega,
    const double dt)
  {
    // Calculate the quaternion derivative
    math::Quaterniond q_dot = get_quaternion_derivative(q, omega);

    // Integrate the quaternion manually (Euler step)
    math::Quaterniond q_integrated(
      q.W() + q_dot.W() * dt,
      q.X() + q_dot.X() * dt,
      q.Y() + q_dot.Y() * dt,
      q.Z() + q_dot.Z() * dt);

    // Normalize to prevent numerical drift
    q_integrated.Normalize();

    return q_integrated;
  }
};

//////////////////////////////////////////////////
OdometryPublisher::OdometryPublisher()
: dataPtr(std::make_unique<OdometryPublisherPrivate>())
{
  std::get<0>(this->dataPtr->linearMean).SetWindowSize(10);
  std::get<1>(this->dataPtr->linearMean).SetWindowSize(10);
  std::get<2>(this->dataPtr->angularMean).SetWindowSize(10);
  std::get<0>(this->dataPtr->linearMean).Clear();
  std::get<1>(this->dataPtr->linearMean).Clear();
  std::get<2>(this->dataPtr->angularMean).Clear();

  if (this->dataPtr->dimensions == 3) {
    std::get<2>(this->dataPtr->linearMean).SetWindowSize(10);
    std::get<0>(this->dataPtr->angularMean).SetWindowSize(10);
    std::get<1>(this->dataPtr->angularMean).SetWindowSize(10);
    std::get<2>(this->dataPtr->linearMean).Clear();
    std::get<0>(this->dataPtr->angularMean).Clear();
    std::get<1>(this->dataPtr->angularMean).Clear();
  }
}

//////////////////////////////////////////////////
//! [Configure]
void OdometryPublisher::Configure(
  const Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  EntityComponentManager & _ecm,
  EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  //! [Configure]

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
    this->dataPtr->offset.Pos() = _sdf->Get<gz::math::Vector3d>(
      "xyz_offset");
  }

  if (_sdf->HasElement("rpy_offset")) {
    this->dataPtr->offset.Rot() =
      gz::math::Quaterniond(
      _sdf->Get<gz::math::Vector3d>(
        "rpy_offset"));
  }

  if (_sdf->HasElement("gaussian_noise")) {
    this->dataPtr->gaussianNoise = _sdf->Get<double>("gaussian_noise");
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
    //! [definePub]
    this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopicValid);
    //! [definePub]
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
}

//////////////////////////////////////////////////
void OdometryPublisher::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("OdometryPublisher::PreUpdate");

  // \TODO(anyone) This is a temporary fix for
  // gazebosim/gz-sim#2165 until gazebosim/gz-sim#2217 is resolved.
  if (kNullEntity == this->dataPtr->model.Entity()) {
    return;
  }

  if (!this->dataPtr->model.Valid(_ecm)) {
    gzwarn << "OdometryPublisher model no longer valid. "
           << "Disabling plugin." << std::endl;
    this->dataPtr->model = Model(kNullEntity);
    return;
  }

  // \TODO(anyone) Support rewind
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

  // \TODO(anyone) This is a temporary fix for
  // gazebosim/gz-sim#2165 until gazebosim/gz-sim#2217 is resolved.
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
  // If not initialized, store initial state and return.
  if (!this->initialized) {
    this->lastUpdateTime = std::chrono::steady_clock::time_point(_info.simTime);
    this->initialPose = worldPose(this->model.Entity(), _ecm);
    this->lastUpdatePose = initialPose;
    this->lastUpdatePoseOdom = initialPose;
    this->initialized = true;
    return;
  }

  // Declare odometry messages.
  msgs::Odometry odometry_msg;
  msgs::Odometry odometry_cov_msg;

  // Compute the time difference since last update.
  const std::chrono::duration<double> dt =
    std::chrono::steady_clock::time_point(_info.simTime) - lastUpdateTime;

  // Avoid zero-time interval updates to prevent division by zero.
  if (math::equal(0.0, dt.count())) {
    return;
  }

  // Set the time stamp in the header.
  msgs::Header header;
  header.mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));

  // Set the frame ids.
  auto frame = header.add_data();
  frame->set_key("frame_id");
  frame->add_value(odomFrame);
  auto childFrame = header.add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(robotBaseFrame);

  // Copy the header to the odometry messages.
  odometry_msg.mutable_header()->CopyFrom(header);
  odometry_cov_msg.mutable_header()->CopyFrom(header);

  // Get the current pose of the robot in the world frame.
  const math::Pose3d rawPose = worldPose(this->model.Entity(), _ecm);
  math::Pose3d pose = rawPose * this->offset;

  // Set pose data in the odometry message.
  odometry_msg.mutable_pose()->mutable_position()->set_x(pose.Pos().X());
  odometry_msg.mutable_pose()->mutable_position()->set_y(pose.Pos().Y());
  msgs::Set(odometry_msg.mutable_pose()->mutable_orientation(), pose.Rot());

  if (this->dimensions == 3) {
    odometry_msg.mutable_pose()->mutable_position()->set_z(pose.Pos().Z());
  }

  // Compute linear displacement.
  double linearDisplacementX = odometry_msg.pose().position().x() -
    this->lastUpdatePose.Pos().X();
  double linearDisplacementY = odometry_msg.pose().position().y() -
    this->lastUpdatePose.Pos().Y();

  // Compute angular displacement.
  double currentYaw = pose.Rot().Yaw();
  const double yawDiff = this->angleMinError(currentYaw, this->lastUpdatePose.Rot().Yaw());

  // Get velocities assuming 2D
  if (this->dimensions == 2) {
    // TODO(fjanguita): Use yawDiff
    double linearVelocityX = (cosf(currentYaw) * linearDisplacementX +
      sinf(currentYaw) * linearDisplacementY) / dt.count();
    double linearVelocityY = (cosf(currentYaw) * linearDisplacementY -
      sinf(currentYaw) * linearDisplacementX) / dt.count();
    std::get<0>(this->linearMean).Push(linearVelocityX);
    std::get<1>(this->linearMean).Push(linearVelocityY);
    odometry_msg.mutable_twist()->mutable_linear()->set_x(
      std::get<0>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    odometry_msg.mutable_twist()->mutable_linear()->set_y(
      std::get<1>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    odometry_msg.mutable_twist()->mutable_linear()->set_z(
      gz::math::Rand::DblNormal(0, this->gaussianNoise));

    odometry_msg.mutable_twist()->mutable_angular()->set_x(
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    odometry_msg.mutable_twist()->mutable_angular()->set_y(
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
  } else if (this->dimensions == 3) {
    double linearDisplacementZ =
      pose.Pos().Z() - this->lastUpdatePose.Pos().Z();

    const double rollDiff = this->angleMinError(
      pose.Rot().Roll(), this->lastUpdatePose.Rot().Roll());
    const double pitchDiff = this->angleMinError(
      pose.Rot().Pitch(), this->lastUpdatePose.Rot().Pitch());

    // Compute linear velocity in world frame.
    math::Vector3 linearVelocity(linearDisplacementX, linearDisplacementY,
      linearDisplacementZ);
    linearVelocity /= dt.count();

    // Transform velocity to robot frame.
    linearVelocity = pose.Rot().RotateVectorReverse(linearVelocity);

    // Store computed velocity for averaging.
    std::get<0>(this->linearMean).Push(linearVelocity.X());
    std::get<1>(this->linearMean).Push(linearVelocity.Y());
    std::get<2>(this->linearMean).Push(linearVelocity.Z());
    std::get<0>(this->angularMean).Push(rollDiff / dt.count());
    std::get<1>(this->angularMean).Push(pitchDiff / dt.count());

    // Set odometry twist linear in robot frame
    odometry_msg.mutable_twist()->mutable_linear()->set_x(
      std::get<0>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    odometry_msg.mutable_twist()->mutable_linear()->set_y(
      std::get<1>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    odometry_msg.mutable_twist()->mutable_linear()->set_z(
      std::get<2>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));

    // Set odometry twist angular in robot frame
    odometry_msg.mutable_twist()->mutable_angular()->set_x(
      std::get<0>(this->angularMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    odometry_msg.mutable_twist()->mutable_angular()->set_y(
      std::get<1>(this->angularMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
  }

  // Update yaw rate.
  std::get<2>(this->angularMean).Push(yawDiff / dt.count());

  odometry_msg.mutable_twist()->mutable_angular()->set_z(
    std::get<2>(this->angularMean).Mean() +
    gz::math::Rand::DblNormal(0, this->gaussianNoise));

  // Obtain linear and angular velocities
  double vx = odometry_msg.mutable_twist()->linear().x();
  double vy = odometry_msg.mutable_twist()->linear().y();
  double vz = odometry_msg.mutable_twist()->linear().z();
  double wx = odometry_msg.mutable_twist()->angular().x();
  double wy = odometry_msg.mutable_twist()->angular().y();
  double wz = odometry_msg.mutable_twist()->angular().z();

  // Compute new position based on linear velocities
  math::Vector3d deltaPosition(
    vx * dt.count(),
    vy * dt.count(),
    this->dimensions == 3 ? vz * dt.count() : 0.0  // If 3D, use Z as well
  );

  // Compute new orientation based on angular velocities
  // math::Quaterniond current_orientation = msgs::Convert(odometry_cov_msg.pose().orientation());
  // math::Vector3d angular_velocity(wx, wy, wz);
  // auto q_integrated = get_quaternion_integrate(
  //   current_orientation, angular_velocity, dt.count());
  double angle = std::sqrt(wx * wx + wy * wy + wz * wz) * dt.count();
  math::Vector3d axis(wx, wy, wz);
  if (angle > 1e-6) {
    axis.Normalize();
  }
  math::Quaterniond deltaOrientation(
    std::cos(angle / 2.0),
    axis.X() * std::sin(angle / 2.0),
    axis.Y() * std::sin(angle / 2.0),
    axis.Z() * std::sin(angle / 2.0)
  );
  math::Quaterniond q_integrated = lastUpdatePoseOdom.Rot() * deltaOrientation;

  // Compute new rotated position
  math::Quaterniond rotation = lastUpdatePoseOdom.Rot();
  rotation.Normalize();
  math::Vector3d deltaPositionRot = rotation.RotateVector(deltaPosition);

  // Update pose
  math::Pose3d updatedPose = lastUpdatePoseOdom;
  updatedPose.Pos() += deltaPositionRot;  // Update position
  updatedPose.Rot() = q_integrated;  // Update rotation

  // Update odometry pose
  odometry_cov_msg.mutable_pose()->mutable_position()->set_x(updatedPose.Pos().X());
  odometry_cov_msg.mutable_pose()->mutable_position()->set_y(updatedPose.Pos().Y());
  if (this->dimensions == 3) {
    odometry_cov_msg.mutable_pose()->mutable_position()->set_z(updatedPose.Pos().Z());
  }
  msgs::Set(odometry_cov_msg.mutable_pose()->mutable_orientation(), updatedPose.Rot());

  math::Pose3d lastUpdatePoseOdom = updatedPose;
  this->lastUpdatePoseOdom = updatedPose;
  this->lastUpdatePose = pose;
  this->lastUpdateTime = std::chrono::steady_clock::time_point(_info.simTime);

  // Throttle publishing.
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
    diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;
  if (this->odomPub.Valid()) {
    this->odomPub.Publish(odometry_msg);
  }

  // Generate odometry with covariance message and publish it
  msgs::OdometryWithCovariance msgCovariance;
  msgCovariance.mutable_header()->CopyFrom(header);

  // Position
  math::Vector3d world_position(
    odometry_cov_msg.pose().position().x(),
    odometry_cov_msg.pose().position().y(),
    odometry_cov_msg.pose().position().z()
  );
  world_position -= initialPose.Pos();
  math::Vector3d odom_position = initialPose.Rot().Inverse().RotateVector(world_position);

  msgCovariance.mutable_pose_with_covariance()->
  mutable_pose()->mutable_position()->set_x(odom_position.X());
  msgCovariance.mutable_pose_with_covariance()->
  mutable_pose()->mutable_position()->set_y(odom_position.Y());
  msgCovariance.mutable_pose_with_covariance()->
  mutable_pose()->mutable_position()->set_z(odom_position.Z());

  // Orientation
  math::Quaterniond world_q;
  world_q.SetX(odometry_cov_msg.pose().orientation().x());
  world_q.SetY(odometry_cov_msg.pose().orientation().y());
  world_q.SetZ(odometry_cov_msg.pose().orientation().z());
  world_q.SetW(odometry_cov_msg.pose().orientation().w());
  math::Quaterniond odom_q = world_q * initialPose.Rot().Inverse();

  msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  mutable_orientation()->set_x(odom_q.X());
  msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  mutable_orientation()->set_y(odom_q.Y());
  msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  mutable_orientation()->set_z(odom_q.Z());
  msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  mutable_orientation()->set_w(odom_q.W());

  // Angular twist Covariance
  msgCovariance.mutable_twist_with_covariance()->
  mutable_twist()->mutable_angular()->set_x(odometry_msg.twist().angular().x());
  msgCovariance.mutable_twist_with_covariance()->
  mutable_twist()->mutable_angular()->set_y(odometry_msg.twist().angular().y());
  msgCovariance.mutable_twist_with_covariance()->
  mutable_twist()->mutable_angular()->set_z(odometry_msg.twist().angular().z());

  // Linear twist
  math::Vector3d base_link_linear_twist(
    odometry_msg.twist().linear().x(),
    odometry_msg.twist().linear().y(),
    odometry_msg.twist().linear().z()
  );
  msgCovariance.mutable_twist_with_covariance()->
  mutable_twist()->mutable_linear()->set_x(base_link_linear_twist.X());
  msgCovariance.mutable_twist_with_covariance()->
  mutable_twist()->mutable_linear()->set_y(base_link_linear_twist.Y());
  msgCovariance.mutable_twist_with_covariance()->
  mutable_twist()->mutable_linear()->set_z(base_link_linear_twist.Z());


  // Populate the covariance matrix.
  // Compute the current position values from odometry message
  double currentX = odometry_cov_msg.pose().position().x();
  double currentY = odometry_cov_msg.pose().position().y();
  double currentZ = (this->dimensions == 3) ? odometry_cov_msg.pose().position().z() : 0.0;

  // Extract current orientation (roll, pitch, yaw) from the quaternion
  gz::math::Quaterniond orientation = gz::msgs::Convert(odometry_cov_msg.pose().orientation());
  double odometry_cov_msgRoll = orientation.Roll();
  double odometry_cov_msgPitch = orientation.Pitch();
  double odometry_cov_msgYaw = orientation.Yaw();

  // Update sums for incremental position variance computation
  sumX += currentX;
  sumY += currentY;
  sumZ += currentZ;
  sumX2 += currentX * currentX;
  sumY2 += currentY * currentY;
  sumZ2 += currentZ * currentZ;

  // Update sums for incremental orientation variance computation
  sumRoll += odometry_cov_msgRoll;
  sumPitch += odometry_cov_msgPitch;
  sumYaw += odometry_cov_msgYaw;
  sumRoll2 += odometry_cov_msgRoll * odometry_cov_msgRoll;
  sumPitch2 += odometry_cov_msgPitch * odometry_cov_msgPitch;
  sumYaw2 += odometry_cov_msgYaw * odometry_cov_msgYaw;

  sampleCount++;

  // Compute dynamic variances for position
  double varianceX =
    (sampleCount > 1) ? (sumX2 / sampleCount - (sumX / sampleCount) * (sumX / sampleCount)) : 0.0;
  double varianceY =
    (sampleCount > 1) ? (sumY2 / sampleCount - (sumY / sampleCount) * (sumY / sampleCount)) : 0.0;
  double varianceZ =
    (this->dimensions == 3 &&
    sampleCount > 1) ? (sumZ2 / sampleCount - (sumZ / sampleCount) * (sumZ / sampleCount)) : 0.0;

  // Compute dynamic variances for orientation (roll, pitch, yaw)
  double varianceRoll =
    (sampleCount >
    1) ? (sumRoll2 / sampleCount - (sumRoll / sampleCount) * (sumRoll / sampleCount)) : 0.0;
  double variancePitch =
    (sampleCount >
    1) ? (sumPitch2 / sampleCount - (sumPitch / sampleCount) * (sumPitch / sampleCount)) : 0.0;
  double varianceYaw =
    (sampleCount >
    1) ? (sumYaw2 / sampleCount - (sumYaw / sampleCount) * (sumYaw / sampleCount)) : 0.0;

  // Store the variance values in the diagonal elements of the covariance matrix
  std::vector<double> diagonalValues = {
    varianceX, varianceY, varianceZ,
    varianceRoll, variancePitch, varianceYaw
  };

  // Populate the covariance matrix with the computed variances
  for (int i = 0; i < 36; i++) {
    if (i % 7 == 0) {
      // Assign computed variance values to the diagonal
      int diagonalIndex = i / 7;
      double value = (diagonalIndex < diagonalValues.size()) ? diagonalValues[diagonalIndex] : 0.0;

      msgCovariance.mutable_pose_with_covariance()->mutable_covariance()->add_data(value);
      msgCovariance.mutable_twist_with_covariance()->mutable_covariance()->add_data(value);
    } else {
      // Assign 0 to non-diagonal elements
      msgCovariance.mutable_pose_with_covariance()->mutable_covariance()->add_data(0.0);
      msgCovariance.mutable_twist_with_covariance()->mutable_covariance()->add_data(0.0);
    }
  }


  // Publish odometry with covariance message
  if (this->odomCovPub.Valid()) {
    this->odomCovPub.Publish(msgCovariance);
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
  "gz::sim::systems::OdometryPublisherDevel")
