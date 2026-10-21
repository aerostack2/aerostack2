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
 * @file rates_geometric_controller.hpp
 *
 * Rates Controller definition.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLERS__RATES_GEOMETRIC_CONTROLLER_HPP_
#define GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLERS__RATES_GEOMETRIC_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>

namespace geometric_controllers
{

/**
 * @brief Geometric controller parameters
 *
 * @tparam P Precision type of the controller
 */
template<typename P = double>
struct RatesGeometricControllerParameters
{
  // Check if P is a numeric type
  static_assert(std::is_floating_point<P>::value, "P must be used with a floating-point type");

  using Vector3 = Eigen::Matrix<P, 3, 1>;

  Vector3 kp_rotation = Vector3::Zero();  // proportional gains for the rotation
};

/**
 * @brief Rates Geometric controller class
 *
 * Convert a thrust and attitude into a desired thrust and body rates using differential flatness
 * properties.
 *
 * @tparam P Precision type of the controller
 */
template<typename P = double>
class RatesGeometricController
{
  // Check if P is a numeric type
  static_assert(std::is_floating_point<P>::value, "P must be used with a floating-point type");

  using Scalar = P;
  using Vector3 = Eigen::Matrix<P, 3, 1>;
  using Matrix3 = Eigen::Matrix<P, 3, 3>;
  using Quaternion = Eigen::Quaternion<P>;

public:
  /**
   * @brief Construct a new RatesGeometricController object
   *
   * @param kp_rotation Vector3 with the proportional gains for the rotation.
   */
  explicit RatesGeometricController(const Vector3 & kp_rotation)
  {
    kp_rotation_ = kp_rotation;
    kp_rotation_mat_ = kp_rotation_.asDiagonal();
  }

  /**
   * @brief Construct a new RatesGeometricController object
   *
   * @param parameters RatesGeometricControllerParameters
   */
  explicit RatesGeometricController(
    const RatesGeometricControllerParameters<P> & parameters =
    RatesGeometricControllerParameters<P>())
  {
    kp_rotation_ = parameters.kp_rotation;
    kp_rotation_mat_ = kp_rotation_.asDiagonal();
  }

  /**
   * @brief Destroy the RatesGeometricController object
   *
   */
  ~RatesGeometricController() {}

  /**
   * @brief Convert a desired attitude reference (thrust and attitude) into a
   * desired body rates reference (thrust and angular velocity)
   *
   * @param desired_thrust Scalar with the desired thrust (N) in z-axis of body frame
   * @param desired_attitude Quaternion with the desired attitude of body frame in earth frame
   * @param current_attitude Quaternion with the current attitude of body frame in earth frame
   *
   * @return std::pair<Scalar, Vector3> Pair with the desired thrust (N) in z-axis of body frame and
   * angular velocity (rad/s) in body frame
   */
  std::pair<Scalar, Vector3> attitude_to_body_rates(
    const Scalar & desired_thrust,
    const Quaternion & desired_attitude,
    const Quaternion & current_attitude)
  {
    thrust_b_ = desired_thrust;

    // Compute the angular velocity
    const Vector3 body_rates = compute_desired_body_rates(desired_attitude, current_attitude);

    return std::make_pair(desired_thrust, body_rates);
  }

  /**
   * @brief Update kp_rotation
   *
   * @param kp_rotation Vector3 with the proportional gains for the rotation
   */
  inline void update_kp_rotation(const Vector3 & kp_rotation)
  {
    kp_rotation_ = kp_rotation;
    kp_rotation_mat_ = kp_rotation_.asDiagonal();
  }

  /**
   * @brief Update controller parameters
   *
   * @param parameters RatesGeometricControllerParameters
   */
  inline void update_parameters(const RatesGeometricControllerParameters<P> & parameters)
  {
    update_kp_rotation(parameters.kp_rotation);
  }

  // Getters

  /**
   * @brief Get the desired reference thrust (N) in z-axis of body frame.
   *
   * @return Scalar Desired thrust (N) in z-axis of body frame
   */
  inline Scalar get_desired_thrust() const {return thrust_b_;}

  /**
   * @brief Get the desired reference body rates with vehicle desired angular velocity
   *
   * @return const Vector3& Desired angular velocity (rad/s)
   */
  inline const Vector3 & get_desired_body_rates() const {return body_rates_;}

  /**
   * @brief Get the kp_rotation
   *
   * @return const Vector3& Proportional gains for the rotation
   */
  inline const Vector3 & get_kp_rotation() const {return kp_rotation_;}

protected:
  // Control
  Vector3 kp_rotation_ = Vector3::Zero();      // Proportional gains for the rotation
  Matrix3 kp_rotation_mat_ = Matrix3::Zero();  // Proportional gains for the rotation

  Scalar thrust_b_ = 0.0;                 // Thrust (N)
  Vector3 body_rates_ = Vector3::Zero();  // Angular velocity (rad/s) in body frame

protected:
  /**
   * @brief Convert a desired attitude (quaternion) in earth frame into a desired angular
   * velocity (rad/s) in body frame
   *
   * @param desired_attitude Quaternion with the desired attitude of body frame in earth frame
   * @param current_attitude Quaternion with the current attitude of body frame in earth frame
   *
   * @return Vector3 Desired angular velocity (rad/s) in body frame
   */
  Vector3 compute_desired_body_rates(
    const Quaternion & desired_attitude,
    const Quaternion & current_attitude)
  {
    // Compute rotation error
    const Quaternion q_des = desired_attitude.normalized();
    const Quaternion q = current_attitude.normalized();
    const Matrix3 R_des = q_des.toRotationMatrix();
    const Matrix3 R = q.toRotationMatrix();
    const Matrix3 Mat_R_error = R_des.transpose() * R - R.transpose() * R_des;

    // Compute rotation vector error
    const Vector3 Vec_R_error = Vector3(Mat_R_error(2, 1), Mat_R_error(0, 2), Mat_R_error(1, 0));

    // Compute the rotation error
    const Vector3 rotation_error = 0.5 * Vec_R_error;

    // Compute the angular velocity error
    body_rates_ = -kp_rotation_mat_ * rotation_error;

    return body_rates_;
  }
};
}  // namespace geometric_controllers

#endif  // GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLERS__RATES_GEOMETRIC_CONTROLLER_HPP_
