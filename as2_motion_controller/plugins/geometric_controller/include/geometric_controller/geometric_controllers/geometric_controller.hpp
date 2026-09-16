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
 * @file geometric_controller.hpp
 *
 * Geometric Controller definition.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLERS__GEOMETRIC_CONTROLLER_HPP_
#define GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLERS__GEOMETRIC_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>

#include "geometric_controllers/attitude_geometric_controller.hpp"
#include "geometric_controllers/rates_geometric_controller.hpp"

namespace geometric_controllers
{

/**
 * @brief Geometric controller parameters
 *
 * @tparam P Precision type of the controller
 */
template<typename P = double>
struct GeometricControllerParameters
{
  // Check if P is a numeric type
  static_assert(std::is_floating_point<P>::value, "P must be used with a floating-point type");

  AttitudeGeometricControllerParameters<P> attitude_parameters =
    AttitudeGeometricControllerParameters<P>();
  RatesGeometricControllerParameters<P> rates_parameters = RatesGeometricControllerParameters<P>();
};

/**
 * @brief Geometric controller class
 *
 * Convert a desired trajectory into a desired attitude and thrust using differential flatness.
 *
 * @tparam P Precision type of the controller
 */
template<typename P = double>
class GeometricController : public AttitudeGeometricController<P>,
  public RatesGeometricController<P>
{
  // Check if P is a numeric type
  static_assert(
    std::is_floating_point<P>::value,
    "GeometricController must be used with a floating-point type");

  using Scalar = P;
  using Vector3 = Eigen::Matrix<P, 3, 1>;
  using Matrix3 = Eigen::Matrix<P, 3, 3>;
  using Quaternion = Eigen::Quaternion<P>;

public:
  /**
   * @brief Construct a new GeometricController object
   *
   * @param AttitudeGeometricControllerParameters Attitude controller parameters
   * @param RatesGeometricControllerParameters Rates controller parameters
   */
  GeometricController(
    AttitudeGeometricControllerParameters<P> attitude_parameters,
    RatesGeometricControllerParameters<P> rates_parameters)
  : AttitudeGeometricController<P>(attitude_parameters), RatesGeometricController<P>(
      rates_parameters) {}

  /**
   * @brief Construct a new GeometricController object
   *
   * @param parameters GeometricControllerParameters
   */
  explicit GeometricController(
    const GeometricControllerParameters<P> & parameters = GeometricControllerParameters<P>())
  : AttitudeGeometricController<P>(parameters.attitude_parameters),
    RatesGeometricController<P>(parameters.rates_parameters) {}

  /**
   * @brief Destroy the GeometricController object
   *
   */
  ~GeometricController() {}

  /**
   * @brief Convert a desired linear acceleration and yaw angle into a desired body rates
   * reference (thrust and angular velocity)
   *
   * @param desired_acceleration Vector3 with the desired acceleration (m/s^2) in earth frame
   * @param desired_yaw Scalar with the desired yaw (rad) in earth frame
   * @param current_attitude Quaternion with the current attitude of body frame in earth frame
   *
   * @return std::pair<Scalar, Vector3> Pair with the desired thrust (N) in z-axis of body frame and
   * angular velocity (rad/s) in body frame
   */
  std::pair<Scalar, Vector3> acceleration_to_rates(
    const Vector3 & desired_acceleration,
    const Scalar desired_yaw,
    const Quaternion & current_attitude)
  {
    // Compute attitude references
    const auto [desired_thrust_b, desired_attitude] =
      AttitudeGeometricController<P>::acceleration_to_attitude(
      desired_acceleration, desired_yaw,
      current_attitude);

    // Compute body rates references
    return RatesGeometricController<P>::attitude_to_body_rates(
      desired_thrust_b, desired_attitude,
      current_attitude);
  }

  /**
   * @brief Update controller parameters
   *
   * @param parameters GeometricControllerParameters
   */
  inline void update_parameters(const GeometricControllerParameters<P> & parameters)
  {
    AttitudeGeometricController<P>::update_parameters(parameters.attitude_parameters);
    RatesGeometricController<P>::update_parameters(parameters.rates_parameters);
  }

  // Getters

  /**
   * @brief Get the desired reference thrust (N) in z-axis of body frame.
   *
   * @return Scalar Desired thrust (N) in z-axis of body frame
   */
  inline Scalar get_desired_thrust() const
  {
    return AttitudeGeometricController<P>::get_desired_thrust();
  }
};
}  // namespace geometric_controllers

#endif  // GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLERS__GEOMETRIC_CONTROLLER_HPP_
