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
  /////////////Añadida
  transport::Node::Publisher odomPubmod;

  /// \brief Odometry with covariance message publisher.

public:
  /////////////////Añadida
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
  // Variables acumuladoras para el cálculo incremental de la varianza (declarar como miembros de la clase)
  double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  double sumX2 = 0.0, sumY2 = 0.0, sumZ2 = 0.0;
  double sumRoll = 0.0, sumPitch = 0.0, sumYaw = 0.0;    // Para orientaciones
  double sumRoll2 = 0.0, sumPitch2 = 0.0, sumYaw2 = 0.0;    // Para los cuadrados de las orientaciones
  int sampleCount = 0;     // Conteo de muestras
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
  std::string odomTopicmod{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/odometrymod"};
  std::string odomCovTopicmod{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/odometry_with_covariancemod"};
  if (_sdf->HasElement("odom_topic")) {
    odomTopic = _sdf->Get<std::string>("odom_topic");
  }
  if (_sdf->HasElement("odom_covariance_topic")) {
    odomCovTopic = _sdf->Get<std::string>("odom_covariance_topic");
  }

  std::string odomTopicValid {transport::TopicUtils::AsValidTopic(odomTopic)};
  std::string odomTopicValidmod {transport::TopicUtils::AsValidTopic(odomTopicmod)};
  if (odomTopicValid.empty()) {
    gzerr << "Failed to generate odom topic ["
          << odomTopic << "]" << std::endl;
  } else {
    //! [definePub]
    this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopicValid);
    this->dataPtr->odomPubmod = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopicValidmod);
    //! [definePub]
    gzmsg << "OdometryPublisher publishing odometry on [" << odomTopicValid
          << "]" << std::endl;
    gzmsg << "OdometryPublisher publishing odometry on [" << odomTopicValidmod
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
  // Record start time.
  if (!this->initialized) {
    this->lastUpdateTime = std::chrono::steady_clock::time_point(_info.simTime);
    this->initialPose = worldPose(this->model.Entity(), _ecm);
    this->lastUpdatePose = initialPose;
    this->lastUpdatePoseOdom = initialPose;
    this->initialized = true;
    return;
  }

  // Construct the odometry message and publish it.
  //! [declarePoseMsg]
  msgs::Odometry msg;
  msgs::Odometry msgm;
  //! [declarePoseMsg]

  const std::chrono::duration<double> dt =
    std::chrono::steady_clock::time_point(_info.simTime) - lastUpdateTime;
  // We cannot estimate the speed if the time interval is zero (or near
  // zero).
  if (math::equal(0.0, dt.count())) {
    return;
  }

  // Get and set robotBaseFrame to odom transformation.
  //! [worldPose]
  const math::Pose3d rawPose = worldPose(this->model.Entity(), _ecm);
  //! [worldPose]
  //! [setPoseMsg]
  math::Pose3d pose = rawPose * this->offset;
  msg.mutable_pose()->mutable_position()->set_x(pose.Pos().X());
  msg.mutable_pose()->mutable_position()->set_y(pose.Pos().Y());
  msgs::Set(msg.mutable_pose()->mutable_orientation(), pose.Rot());
  if (this->dimensions == 3) {
    msg.mutable_pose()->mutable_position()->set_z(pose.Pos().Z());
  }
  //! [setPoseMsg]

  // Get linear and angular displacements from last updated pose.
  double linearDisplacementX = pose.Pos().X() - this->lastUpdatePose.Pos().X();
  double linearDisplacementY = pose.Pos().Y() - this->lastUpdatePose.Pos().Y();

  double currentYaw = pose.Rot().Yaw();
  const double lastYaw = this->lastUpdatePose.Rot().Yaw();
  while (currentYaw < lastYaw - GZ_PI) {currentYaw += 2 * GZ_PI;}
  while (currentYaw > lastYaw + GZ_PI) {currentYaw -= 2 * GZ_PI;}
  const float yawDiff = currentYaw - lastYaw;

  // Get velocities assuming 2D
  if (this->dimensions == 2) {
    double linearVelocityX = (cosf(currentYaw) * linearDisplacementX +
      sinf(currentYaw) * linearDisplacementY) / dt.count();
    double linearVelocityY = (cosf(currentYaw) * linearDisplacementY -
      sinf(currentYaw) * linearDisplacementX) / dt.count();
    std::get<0>(this->linearMean).Push(linearVelocityX);
    std::get<1>(this->linearMean).Push(linearVelocityY);
    msg.mutable_twist()->mutable_linear()->set_x(
      std::get<0>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_linear()->set_y(
      std::get<1>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_linear()->set_z(
      gz::math::Rand::DblNormal(0, this->gaussianNoise));

    msg.mutable_twist()->mutable_angular()->set_x(
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_angular()->set_y(
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
  }
  // Get velocities and roll/pitch rates assuming 3D
  else if (this->dimensions == 3) {
    double currentRoll = pose.Rot().Roll();
    const double lastRoll = this->lastUpdatePose.Rot().Roll();
    while (currentRoll < lastRoll - GZ_PI) {currentRoll += 2 * GZ_PI;}
    while (currentRoll > lastRoll + GZ_PI) {currentRoll -= 2 * GZ_PI;}
    const float rollDiff = currentRoll - lastRoll;

    double currentPitch = pose.Rot().Pitch();
    const double lastPitch = this->lastUpdatePose.Rot().Pitch();
    while (currentPitch < lastPitch - GZ_PI) {currentPitch += 2 * GZ_PI;}
    while (currentPitch > lastPitch + GZ_PI) {currentPitch -= 2 * GZ_PI;}
    const float pitchDiff = currentPitch - lastPitch;

    double linearDisplacementZ =
      pose.Pos().Z() - this->lastUpdatePose.Pos().Z();
    math::Vector3 linearDisplacement(linearDisplacementX, linearDisplacementY,
      linearDisplacementZ);
    math::Vector3 linearVelocity =
      pose.Rot().RotateVectorReverse(linearDisplacement) / dt.count();
    std::get<0>(this->linearMean).Push(linearVelocity.X());
    std::get<1>(this->linearMean).Push(linearVelocity.Y());
    std::get<2>(this->linearMean).Push(linearVelocity.Z());
    std::get<0>(this->angularMean).Push(rollDiff / dt.count());
    std::get<1>(this->angularMean).Push(pitchDiff / dt.count());
    msg.mutable_twist()->mutable_linear()->set_x(
      std::get<0>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_linear()->set_y(
      std::get<1>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_linear()->set_z(
      std::get<2>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_angular()->set_x(
      std::get<0>(this->angularMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_angular()->set_y(
      std::get<1>(this->angularMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
  }

  // Set yaw rate
  std::get<2>(this->angularMean).Push(yawDiff / dt.count());
  msg.mutable_twist()->mutable_angular()->set_z(
    std::get<2>(this->angularMean).Mean() +
    gz::math::Rand::DblNormal(0, this->gaussianNoise));

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

  msg.mutable_header()->CopyFrom(header);

  // Obtener las velocidades lineales y angulares
  double vx = msg.mutable_twist()->linear().x();
  double vy = msg.mutable_twist()->linear().y();
  double vz = msg.mutable_twist()->linear().z();
  double wx = msg.mutable_twist()->angular().x();
  double wy = msg.mutable_twist()->angular().y();
  double wz = msg.mutable_twist()->angular().z();

  // Calcular la nueva posición basada en las velocidades lineales
  math::Vector3d deltaPosition(
    vx * dt.count(),
    vy * dt.count(),
    this->dimensions == 3 ? vz * dt.count() : 0.0    // Si es 3D, usar también Z
  );

  // Calcular la nueva orientación basada en las velocidades angulares
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

  // Calcular la nueva posición rotada
  math::Quaterniond rotation = lastUpdatePoseOdom.Rot();
  rotation.Normalize();
  math::Vector3d deltaPositionRot = rotation.RotateVector(deltaPosition);
  //deltaOrientation.Normalize(); // Normalizar para evitar errores acumulativos

  //math::Vector3d deltaPositionRot = lastUpdatePoseOdom.Rot().RotateVector(deltaPosition);

  // Sumar el desplazamiento calculado a la última posición y orientación
  math::Pose3d updatedPose = lastUpdatePoseOdom;
  updatedPose.Pos() += deltaPositionRot;  // Sumar desplazamiento lineal
  updatedPose.Rot() = updatedPose.Rot() * deltaOrientation;  // Combinar rotaciones

  // Actualizar el mensaje de pose
  msgm.mutable_pose()->mutable_position()->set_x(updatedPose.Pos().X());
  msgm.mutable_pose()->mutable_position()->set_y(updatedPose.Pos().Y());
  if (this->dimensions == 3) {
    msgm.mutable_pose()->mutable_position()->set_z(updatedPose.Pos().Z());
  }
  msgs::Set(msgm.mutable_pose()->mutable_orientation(), updatedPose.Rot());
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
    //! [publishMsg]
    this->odomPub.Publish(msg);
    //! [publishMsg]
  }
  if (this->odomPubmod.Valid()) {
    //! [publishMsg]
    this->odomPubmod.Publish(msgm);
    //! [publishMsgm]
  }
  // Get and set robotBaseFrame to odom transformation.
  // Generate odometry with covariance message and publish it.
  msgs::OdometryWithCovariance msgCovariance;

  // Set the time stamp in the header.
  msgCovariance.mutable_header()->CopyFrom(header);

  // Copy position from odometry msg.
  msgCovariance.mutable_pose_with_covariance()->
  mutable_pose()->mutable_position()->set_x(
    msgm.pose().position().x() -
    this->initialPose.Pos().X());
  msgCovariance.mutable_pose_with_covariance()->
  mutable_pose()->mutable_position()->set_y(
    msgm.pose().position().y() -
    this->initialPose.Pos().Y());
  msgCovariance.mutable_pose_with_covariance()->
  mutable_pose()->mutable_position()->set_z(
    msg.pose().position().z() -
    this->initialPose.Pos().Z());

  // msgCovariance.mutable_pose_with_covariance()->
  // mutable_pose()->mutable_position()->set_x(
  //   msgm.pose().position().x());
  // msgCovariance.mutable_pose_with_covariance()->
  // mutable_pose()->mutable_position()->set_y(
  //   msgm.pose().position().y());
  // msgCovariance.mutable_pose_with_covariance()->
  // mutable_pose()->mutable_position()->set_z(
  //   msg.pose().position().z());

  // Copy orientation from odometry msg.
  //msgs::Set(
  //msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  //mutable_orientation(), pose.Rot());
  // Copiar orientación directamente desde el mensaje de odometría

  // Compute quaternion in odom frame
  math::Quaterniond quat_earth;

  quat_earth.SetX(msgm.pose().orientation().x());
  quat_earth.SetY(msgm.pose().orientation().y());
  quat_earth.SetZ(msgm.pose().orientation().z());
  quat_earth.SetW(msgm.pose().orientation().w());

  math::Quaterniond quat_odom = quat_earth * initialPose.Rot().Inverse();

  msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  mutable_orientation()->set_x(quat_odom.X());
  msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  mutable_orientation()->set_y(quat_odom.Y());
  msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  mutable_orientation()->set_z(quat_odom.Z());
  msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  mutable_orientation()->set_w(quat_odom.W());

  // msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  // mutable_orientation()->set_x(msgm.pose().orientation().x());
  // msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  // mutable_orientation()->set_y(msgm.pose().orientation().y());
  // msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  // mutable_orientation()->set_z(msgm.pose().orientation().z());
  // msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
  // mutable_orientation()->set_w(msgm.pose().orientation().w());

  // Copy twist from odometry msg.
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

  // Populate the covariance matrix.
  // Should the matrix me populated for pose as well ?
  // Obtener las posiciones actuales del mensaje msgm
  double currentX = msgm.pose().position().x();
  double currentY = msgm.pose().position().y();
  double currentZ = this->dimensions == 3 ? msgm.pose().position().z() : 0.0;

  // Obtener las orientaciones actuales (roll, pitch, yaw) desde el quaternion
  gz::math::Quaterniond orientation = gz::msgs::Convert(msgm.pose().orientation());
  double msgmRoll = orientation.Roll();
  double msgmPitch = orientation.Pitch();
  double msgmYaw = orientation.Yaw();

  // Actualizar las sumas para el cálculo incremental de las posiciones
  sumX += currentX;
  sumY += currentY;
  sumZ += currentZ;

  sumX2 += currentX * currentX;
  sumY2 += currentY * currentY;
  sumZ2 += currentZ * currentZ;

  // Actualizar las sumas para el cálculo incremental de las orientaciones
  sumRoll += msgmRoll;
  sumPitch += msgmPitch;
  sumYaw += msgmYaw;

  sumRoll2 += msgmRoll * msgmRoll;
  sumPitch2 += msgmPitch * msgmPitch;
  sumYaw2 += msgmYaw * msgmYaw;

  sampleCount++;

  // Calcular las varianzas dinámicas para las posiciones
  double varianceX = (sampleCount > 1) ?
    (sumX2 / sampleCount - (sumX / sampleCount) * (sumX / sampleCount)) : 0.0;
  double varianceY = (sampleCount > 1) ?
    (sumY2 / sampleCount - (sumY / sampleCount) * (sumY / sampleCount)) : 0.0;
  double varianceZ = (this->dimensions == 3 && sampleCount > 1) ?
    (sumZ2 / sampleCount - (sumZ / sampleCount) * (sumZ / sampleCount)) : 0.0;

  // Calcular las varianzas dinámicas para las orientaciones (roll, pitch, yaw)
  double varianceRoll = (sampleCount > 1) ?
    (sumRoll2 / sampleCount - (sumRoll / sampleCount) * (sumRoll / sampleCount)) : 0.0;
  double variancePitch = (sampleCount > 1) ?
    (sumPitch2 / sampleCount - (sumPitch / sampleCount) * (sumPitch / sampleCount)) : 0.0;
  double varianceYaw = (sampleCount > 1) ?
    (sumYaw2 / sampleCount - (sumYaw / sampleCount) * (sumYaw / sampleCount)) : 0.0;

  // Valores concretos para los elementos diagonales de la matriz de covarianza
  std::vector<double> diagonalValues = {
    varianceX, varianceY, varianceZ,
    varianceRoll, variancePitch, varianceYaw
  };

  // Llenar la matriz de covarianza
  for (int i = 0; i < 36; i++) {
    if (i % 7 == 0) {
      // Asignar un valor concreto de la lista diagonalValues
      int diagonalIndex = i / 7;
      double value = diagonalIndex < diagonalValues.size() ? diagonalValues[diagonalIndex] : 0.0;

      msgCovariance.mutable_pose_with_covariance()->
      mutable_covariance()->add_data(value);
      msgCovariance.mutable_twist_with_covariance()->
      mutable_covariance()->add_data(value);
    } else {
      // Fuera de la diagonal principal, asignar 0
      msgCovariance.mutable_pose_with_covariance()->
      mutable_covariance()->add_data(0.0);
      msgCovariance.mutable_twist_with_covariance()->
      mutable_covariance()->add_data(0.0);
    }
  }

  // Publicar el mensaje de covarianza
  if (this->odomCovPub.Valid()) {
    this->odomCovPub.Publish(msgCovariance);
  }

  // Publicar transformación (tf) si es válido
  if (this->tfPub.Valid()) {
    msgs::Pose_V tfMsg;
    auto tfMsgPose = tfMsg.add_pose();
    tfMsgPose->CopyFrom(msg.pose());
    tfMsgPose->mutable_header()->CopyFrom(header);

    // this->tfPub.Publish(tfMsg);
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
