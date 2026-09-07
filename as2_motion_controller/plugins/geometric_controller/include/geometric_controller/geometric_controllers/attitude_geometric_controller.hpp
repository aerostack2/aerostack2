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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
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
 * @file attitude_geometric_controller.hpp
 *
 * Attitude Geometric Controller definition.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLERS__ATTITUDE_GEOMETRIC_CONTROLLER_HPP_
#define GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLERS__ATTITUDE_GEOMETRIC_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>

namespace geometric_controllers
{

/**
 * @brief Attitude Geometric controller parameters
 *
 * @tparam P Precision type of the controller
 */
template<typename P = double>
struct AttitudeGeometricControllerParameters
{
  // Check if P is a numeric type
  static_assert(std::is_floating_point<P>::value, "P must be used with a floating-point type");

  using Scalar = P;
  using Vector3 = Eigen::Matrix<P, 3, 1>;

  Scalar vehicle_mass = 0.0;   // kg
  Scalar gravity = 9.81;       // m/s^2
};

/**
 * @brief Attitude Geometric controller class
 *
 * Convert a desired acceleration into a desired attitude and thrust using geometric control.
 *
 * @tparam P Precision type of the controller
 */
template<typename P = double>
class AttitudeGeometricController
{
  // Check if P is a numeric type
  static_assert(std::is_floating_point<P>::value, "P must be used with a floating-point type");

  using Scalar = P;
  using Vector3 = Eigen::Matrix<P, 3, 1>;
  using Matrix3 = Eigen::Matrix<P, 3, 3>;
  using Quaternion = Eigen::Quaternion<P>;

public:
  /**
   * @brief Construct a new AttitudeGeometricController object
   *
   * @param vehicle_mass Scalar with the vehicle mass
   * @param gravity Scalar with the gravity. Default: 9.81 (m/s^2)
   */
  explicit AttitudeGeometricController(Scalar vehicle_mass, Scalar gravity = 9.81)
  : vehicle_mass_(vehicle_mass)
  {
    gravity_ = Vector3(0.0, 0.0, gravity);
  }

  /**
   * @brief Construct a new AttitudeGeometricController object
   *
   * @param parameters AttitudeGeometricControllerParameters
   */
  explicit AttitudeGeometricController(
    const AttitudeGeometricControllerParameters<P> & parameters =
    AttitudeGeometricControllerParameters<P>())
  : vehicle_mass_(parameters.vehicle_mass)
  {
    gravity_ = Vector3(0.0, 0.0, parameters.gravity);
  }

  /**
   * @brief Destroy the AttitudeGeometricController object
   *
   */
  ~AttitudeGeometricController() {}

  /**
   * @brief Convert a desired linear acceleration and yaw angle into a desired attitude and thrust
   *
   * @param desired_acceleration Vector3 with the desired acceleration (m/s^2) in earth frame
   * @param desired_yaw Scalar with the desired yaw (rad) in earth frame
   * @param current_attitude Quaternion with the current attitude of body frame in earth frame
   *
   * @return std::pair<Scalar, Quaternion> Pair with the desired thrust (N) in z-axis of body frame
   * and attitude (quaternion) in earth frame
   */
  std::pair<Scalar, Quaternion> acceleration_to_attitude(
    const Vector3 & desired_acceleration,
    const Scalar desired_yaw,
    const Quaternion & current_attitude)
  {
    // Compute thrust
    const Vector3 desired_thrust = compute_desired_thrust(desired_acceleration);
    const Scalar desired_thrust_b = compute_desired_body_thrust(desired_thrust, current_attitude);

    // Compute desired attitude
    Quaternion desired_attitude = compute_desired_attitude(desired_thrust, desired_yaw);

    // Return desired thrust and attitude
    return std::make_pair(desired_thrust_b, desired_attitude);
  }

  /**
   * @brief Update vehicle mass
   *
   * @param vehicle_mass Scalar with the vehicle mass (kg)
   */
  inline void update_vehicle_mass(const Scalar vehicle_mass) {vehicle_mass_ = vehicle_mass;}

  /**
   * @brief Update gravity vector
   *
   * @param gravity Scalar with the gravity.
   */
  inline void update_gravity(const Scalar gravity) {gravity_ = Vector3(0.0, 0.0, gravity);}

  /**
   * @brief Update controller parameters
   *
   * @param parameters AttitudeGeometricControllerParameters
   */
  inline void update_parameters(const AttitudeGeometricControllerParameters<P> & parameters)
  {
    update_vehicle_mass(parameters.vehicle_mass);
    update_gravity(parameters.gravity);
  }

  // Getters

  /**
   * @brief Get the desired reference thrust (N) in z-axis of body frame.
   *
   * @return Scalar Desired thrust (N) in z-axis of body frame
   */
  inline Scalar get_desired_thrust() const {return thrust_b_;}

  /**
   * @brief Get the desired reference thrust (N) in earth frame.
   *
   * @return Vector3 Desired thrust (N) in earth frame
   */
  inline const Vector3 & get_desired_thrust_vector() const {return thrust_;}

  /**
   * @brief Get the desired reference attitude (quaternion) in earth frame.
   *
   * @return const Quaternion& Desired attitude (quaternion) in earth frame
   */
  inline const Quaternion & get_desired_attitude() const {return desired_attitude_;}

  /**
   * @brief Get the vehicle mass
   *
   * @return Scalar Vehicle mass (kg)
   */
  inline Scalar get_vehicle_mass() const {return vehicle_mass_;}

  /**
   * @brief Get the gravity vector
   *
   * @return Scalar Gravity vector (m/s^2)
   */
  inline Scalar get_gravity() const {return gravity_.z();}

protected:
  // Internal variables
  Scalar vehicle_mass_ = 0;                        // Vehicle mass (kg)
  Vector3 gravity_ = Vector3(0.0, 0.0, 9.81);      // Gravity vector (m/s^2)

  Vector3 thrust_ = Vector3::Zero();                      // Thrust (N) in earth frame
  Scalar thrust_b_ = 0.0;                                 // Thrust (N)
  Quaternion desired_attitude_ = Quaternion::Identity();  // Desired attitude

protected:
  /**
   * @brief Compute desired thrust (N) in earth frame from desired acceleration (m/s^2) in earth
   * frame
   *
   * @param desired_acceleration Vector3 with the desired acceleration (m/s^2) in earth frame
   *
   * @return Vector3 Desired thrust (N) in earth frame
   */
  Vector3 compute_desired_thrust(const Vector3 & desired_acceleration)
  {
    thrust_ = (desired_acceleration + gravity_) * vehicle_mass_;
    return thrust_;
  }

  /**
   * @brief Compute desired thrust (N) in z-axis of body frame from desired thrust (N) in earth
   * frame and current attitude (quaternion) in earth frame
   *
   * @param desired_thrust Vector3 with the desired thrust (N) in earth frame
   * @param current_attitude Quaternion with the current attitude of body frame in earth frame
   *
   * @return Scalar Desired thrust (N) in z-axis of body frame
   */
  Scalar compute_desired_body_thrust(
    const Vector3 & desired_thrust,
    const Quaternion & current_attitude)
  {
    thrust_ = current_attitude.inverse() * desired_thrust;    // Transform to body frame
    thrust_b_ = thrust_.z();
    return thrust_b_;
  }

  /**
   * @brief Convert a desired thrust (N) in earth frame and desired yaw (rad) in earth frame into a
   * desired attitude (quaternion) in earth frame
   *
   * @param desired_thrust Vector3 with the desired thrust (N) in earth frame
   * @param desired_yaw Scalar with the desired yaw (rad) in earth frame
   *
   * @return Quaternion Desired attitude (quaternion) in earth frame
   */
  Quaternion compute_desired_attitude(const Vector3 & desired_thrust, const Scalar desired_yaw)
  {
    // Compute desired attitude
    const Vector3 xc_des = Vector3(cos(desired_yaw), sin(desired_yaw), 0.0);
    Vector3 zb_des = desired_thrust;
    zb_des.normalize();
    const Vector3 yb_des = zb_des.cross(xc_des).normalized();
    const Vector3 xb_des = yb_des.cross(zb_des).normalized();

    // Compute desired rotation matrix
    Matrix3 R_des;
    R_des << xb_des, yb_des, zb_des;

    // Get desired attitude as quaternion
    desired_attitude_ = Quaternion(R_des);
    desired_attitude_.normalize();

    return desired_attitude_;
  }
};
}  // namespace geometric_controllers

#endif  // GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLERS__ATTITUDE_GEOMETRIC_CONTROLLER_HPP_
