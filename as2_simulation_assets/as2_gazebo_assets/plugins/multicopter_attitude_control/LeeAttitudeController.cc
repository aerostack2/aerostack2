/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "LeeAttitudeController.hh"
#include <gz/math/eigen3/Conversions.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
namespace multicopter_control
{
//////////////////////////////////////////////////
std::unique_ptr<LeeAttitudeController> LeeAttitudeController::MakeController(
  const LeeAttitudeControllerParameters & _controllerParams,
  const VehicleParameters & _vehicleParams)
{
  // auto controller = std::make_unique<LeeAttitudeController>();
  // Can't use make_unique here because the constructor is private
  std::unique_ptr<LeeAttitudeController> controller(
    new LeeAttitudeController());
  controller->controllerParameters = _controllerParams;
  controller->vehicleParameters = _vehicleParams;
  if (controller->InitializeParameters()) {
    return controller;
  } else {
    return nullptr;
  }
}

//////////////////////////////////////////////////
bool LeeAttitudeController::InitializeParameters()
{
  auto allocationMatrix =
    calculateAllocationMatrix(this->vehicleParameters.rotorConfiguration);
  if (!allocationMatrix.has_value()) {
    // Error should already be printed by function
    return false;
  }

  // To make the tuning independent of the inertia matrix we divide here.
  this->normalizedAttitudeGain =
    this->controllerParameters.attitudeGain.transpose() *
    this->vehicleParameters.inertia.inverse();

  this->normalizedAngularRateGain =
    this->controllerParameters.angularRateGain.transpose() *
    this->vehicleParameters.inertia.inverse();

  Eigen::Matrix4d moi;
  moi.setZero();
  moi.block<3, 3>(0, 0) = this->vehicleParameters.inertia;
  moi(3, 3) = 1;

  this->angularAccToRotorVelocities.resize(
    this->vehicleParameters.rotorConfiguration.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia
  // matrix I. A^{ \dagger} = A^T*(A*A^T)^{-1}
  const auto & aMat = *allocationMatrix;
  this->angularAccToRotorVelocities =
    aMat.transpose() * (aMat * aMat.transpose()).inverse() * moi;

  return true;
}

//////////////////////////////////////////////////
void LeeAttitudeController::CalculateRotorVelocities(
  const FrameData & _frameData, const RollPitchYawRateThrust & _command,
  Eigen::VectorXd & _rotorVelocities) const
{

  // clamp roll and pitch values between -90 and 90 degrees TODO parametrize from SDF
  double roll = std::max(std::min(_command.roll, GZ_PI / 2.0), -GZ_PI / 2.0);
  double pitch = std::max(std::min(_command.pitch, GZ_PI / 2.0), -GZ_PI / 2.0);
  // clamp yaw rate between 180 and -180 degrees per second
  double yawRate = std::max(
    std::min(_command.yawRate, GZ_PI),
    -GZ_PI);
  double thrust = _command.thrust;

  // get current yaw. Need to convert to math::Quaterniond since Eigen::eulerAngles() has a weird behavior with order (2, 1, 0)
  Eigen::Quaterniond currentEigenQuat(_frameData.pose.linear());
  math::Quaterniond currentQuat(currentEigenQuat.w(), currentEigenQuat.x(), currentEigenQuat.y(),
    currentEigenQuat.z());
  double currentYaw = currentQuat.Yaw();

  // define current desired rotation matrix from desired roll, desired pitch and current yaw
  Eigen::Matrix3d rotDes;
  rotDes = Eigen::AngleAxisd(currentYaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  Eigen::Vector3d angularAcceleration =
    this->ComputeDesiredAngularAcc(_frameData, rotDes, yawRate);

  Eigen::Vector4d angularAccelerationThrust;
  angularAccelerationThrust.block<3, 1>(0, 0) = angularAcceleration;
  angularAccelerationThrust(3) = thrust;

  _rotorVelocities =
    this->angularAccToRotorVelocities * angularAccelerationThrust;

  _rotorVelocities =
    _rotorVelocities.cwiseMax(Eigen::VectorXd::Zero(_rotorVelocities.rows()));
  _rotorVelocities = _rotorVelocities.cwiseSqrt();
}

//////////////////////////////////////////////////
// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on
// SE(3)
Eigen::Vector3d LeeAttitudeController::ComputeDesiredAngularAcc(
  const FrameData & _frameData, const Eigen::Matrix3d & _rotDes, const double & _yawRate) const
{
  const Eigen::Matrix3d & rot = _frameData.pose.linear();


  // Angle error according to lee et al.
  Eigen::Matrix3d angleErrorMatrix =
    0.5 * (_rotDes.transpose() * rot - rot.transpose() * _rotDes);
  Eigen::Vector3d angleError = vectorFromSkewMatrix(angleErrorMatrix);

  Eigen::Vector3d angularRateDes(Eigen::Vector3d::Zero());
  angularRateDes[2] = _yawRate;

  // The paper shows
  // e_omega = omega - R.T * R_d * omega_des
  // The code in the RotorS implementation has
  // e_omega = omega - R_d.T * R * omega_des
  Eigen::Vector3d angularRateError = _frameData.angularVelocityBody -
    rot.transpose() * _rotDes * angularRateDes;

  // The following MOI terms are computed in the paper, but the RotorS
  // implementation ignores them. They don't appear to make much of a
  // difference.
  // Eigen::Matrix3d moi = this->vehicleParameters.inertia;
  // const Eigen::Vector3d &omega = _frameData.angularVelocityBody;

  // Eigen::Vector3d moiTerm = omega.cross(moi * omega);

  // Eigen::Vector3d moiTerm2 = moi * (skewMatrixFromVector(omega) *
  //                            rot.transpose() * rotDes * angularRateDes);

  // std::cout << moiTerm2.transpose() << std::endl;
  // return -1 * angleError.cwiseProduct(this->normalizedAttitudeGain) -
  //         angularRateError.cwiseProduct(this->normalizedAngularRateGain) +
  //         moiTerm - moiTerm2;
  return -1 * angleError.cwiseProduct(this->normalizedAttitudeGain) -
         angularRateError.cwiseProduct(this->normalizedAngularRateGain);
}
}  // namespace multicopter_control
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
