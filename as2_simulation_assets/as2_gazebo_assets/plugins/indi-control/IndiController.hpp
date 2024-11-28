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
 *  \file       IndiController.hpp
 *  \brief      IndiController class definition.
 *  \authors    Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__INDI_CONTROL__INDICONTROLLER_HPP_
#define AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__INDI_CONTROL__INDICONTROLLER_HPP_

#include "PIDController.hpp"

namespace indi_controller
{

/**
 * @brief Compute element-wise square root and keep the sign.
 *
 * This function computes the square root of each element of the input vector
 * and retains the original sign of each element.
 *
 * @tparam Vector The type of the input vector (Eigen vector type)
 * @param vector The input vector
 * @return Vector The resulting vector with square root of elements and original sign
 */
template<typename Vector>
Vector sqrt_keep_sign(const Vector & vector)
{
  // Stored the sign of each element
  Vector sign = vector.array().sign();

  // Set each element to its absolute value
  Vector abs_vector = vector.array().abs();

  // Compute the square root of each element
  Vector sqrt_vector = abs_vector.array().sqrt();

  // Multiply each element by its sign
  return sqrt_vector.array() * sign.array();
}

/**
 * @brief INDI controller parameters
 *
 * @tparam P Precision type of the controller
 */
template<typename P = double, int num_rotors = 4>
struct IndiControllerParams
{
  // Check if P is a numeric type
  static_assert(
    std::is_floating_point<P>::value,
    "MotorParams must be used with a floating-point type");

  using Matrix3 = Eigen::Matrix<P, 3, 3>;
  using MatrixN = Eigen::Matrix<P, num_rotors, 4>;
  using PIDParams = pid_controller::PIDParams<P>;

  Matrix3 inertia = Matrix3::Zero();  // Inertia matrix (kg m^2)
  MatrixN mixer_matrix_inverse =
    MatrixN::Zero();                   // Mixer matrix inverse [num_rotors x 6] [Fxyz, Mxyz]
  PIDParams pid_params = PIDParams();  // PID parameters for angular velocity control
};

/**
 * @brief Incremental Nonlinear Dynamic Inversion (INDI) controller
 *
 * Convert the desired thrust and angular velocity to motor angular velocity.
 *
 * @tparam P Precision type of the controller
 * @tparam num_rotors Number of rotors of the multirotor
 */
template<typename P = double, int num_rotors = 4>
class IndiController : public pid_controller::PID<P>
{
  // Check if P is a numeric type
  static_assert(
    std::is_floating_point<P>::value,
    "IndiController must be used with a floating-point type");

  using Scalar = P;
  using Vector3 = Eigen::Matrix<P, 3, 1>;
  using Vector4 = Eigen::Matrix<P, 4, 1>;
  using Vector6 = Eigen::Matrix<P, 6, 1>;
  using VectorN = Eigen::Matrix<P, num_rotors, 1>;
  using Matrix3 = Eigen::Matrix<P, 3, 3>;
  using Matrix4 = Eigen::Matrix<P, 4, 4>;
  using Matrix6 = Eigen::Matrix<P, 6, 6>;
  using MatrixN = Eigen::Matrix<P, num_rotors, 6>;
  using Matrix6N = Eigen::Matrix<P, 6, num_rotors>;
  using PID = pid_controller::PID<P>;
  using PIDParams = pid_controller::PIDParams<P>;

public:
  /**
   * @brief Construct a new Indi Controller object
   *
   * @param inertia Vehicle inertia matrix (kg m^2)
   * @param mixer_matrix_inverse Mixer matrix inverse [num_rotors x 6]
   * @param pid_params PID parameters
   */
  IndiController(
    const Matrix3 & inertia,
    const Matrix4 & mixer_matrix_inverse,
    const PIDParams & pid_params)
  : inertia_(inertia), mixer_matrix_inverse_(mixer_matrix_inverse), PID(pid_params) {}

  /**
   * @brief Construct a new Indi Controller object
   *
   * @param params IndiControllerParams parameters
   */
  explicit IndiController(const IndiControllerParams<P> & params = IndiControllerParams<P>())
  : inertia_(params.inertia), mixer_matrix_inverse_(params.mixer_matrix_inverse),
    PID(params.pid_params) {}
  /**
   * @brief Destroy the Indi Controller object
   *
   */
  ~IndiController() {}

  /**
   * @brief Compute the control action
   *
   * @param current_vehicle_angular_velocity Vector3 with the current vehicle angular velocity
   * (rad/s) in body frame
   * @param thrust Scalar with the desired thrust (N) in body frame
   * @param desired_angular_velocity Vector3 with the desired angular velocity (rad/s) in body frame
   * @param dt Scalar with the time step (s)
   *
   * @return VectorN with the desired motor angular velocity squared (rad^2/s^2)
   */
  VectorN acro_to_motor_angular_velocity(
    const Vector3 & current_vehicle_angular_velocity,
    const Scalar thrust,
    const Vector3 & desired_angular_velocity,
    const Scalar dt)
  {
    // PID control for get the desired angular velocity
    const Vector3 angular_velocity_error =
      this->get_error(current_vehicle_angular_velocity, desired_angular_velocity);

    const Vector3 desired_angular_acceleration = this->compute_control(dt, angular_velocity_error);

    // Compute the desired torque: L = I * dw/dt + w x (I * w)
    desired_torque_ = inertia_ * desired_angular_acceleration +
      desired_angular_velocity.cross(inertia_ * desired_angular_velocity);

    // Compute the desired motor angular velocity squared
    desired_thrust_ = Vector3(0.0, 0.0, thrust);

    Vector4 desired;
    desired << thrust, desired_torque_.x(),
      desired_torque_.y(), desired_torque_.z();

    motor_angular_velocity_ = mixer_matrix_inverse_ * desired;

    // Get the desired motor angular velocity
    motor_angular_velocity_ = sqrt_keep_sign(motor_angular_velocity_);
    return motor_angular_velocity_;
  }

  /**
   * @brief Update inertial
   *
   * @param inertia Matrix3 Inertia matrix (kg m^2)
   */
  inline void update_inertia(const Matrix3 & inertia) {inertia_ = inertia;}

  /**
   * @brief Update mixer matrix inverse
   *
   * @param mixer_matrix_inverse MatrixN Mixer matrix inverse
   */
  inline void update_mixer_matrix_inverse(const Matrix4 & mixer_matrix_inverse)
  {
    mixer_matrix_inverse_ = mixer_matrix_inverse;
  }

  /**
   * @brief Update controller parameters
   *
   * @param params IndiControllerParams
   */
  inline void update_params(const IndiControllerParams<P> & params)
  {
    update_inertia(params.inertia);
    update_mixer_matrix_inverse(params.mixer_matrix_inverse);
    this->update_pid_params(params.pid_params);
  }

  // Getters

  /**
   * @brief Get the inertia
   *
   * @return Matrix3 Inertia matrix (kg m^2)
   */
  inline const Matrix3 & get_inertia() const {return inertia_;}

  /**
   * @brief Get the mixer matrix inverse
   *
   * @return MatrixN Mixer matrix inverse
   */
  inline const Matrix4 & get_mixer_matrix_inverse() const {return mixer_matrix_inverse_;}

  /**
   * @brief Get the desired angular acceleration
   *
   * @return constVector3& Desired angular acceleration (rad/s^2)
   */
  inline const Vector3 & get_desired_angular_acceleration() const {return this->get_output();}

  /**
   * @brief Get the desired thrust
   *
   * @return const Vector3& Desired thrust (N)
   */
  inline const Vector3 & get_desired_thrust() const {return desired_thrust_;}

  /**
   * @brief Get the desired torque
   *
   * @return const Vector3& Desired torque (N m)
   */
  inline const Vector3 & get_desired_torque() const {return desired_torque_;}

  /**
   * @brief Get the motor angular velocity
   *
   * @return const VectorN& Motor angular velocity squared (rad^2/s^2)
   */
  inline const VectorN & get_motor_angular_velocity() const {return motor_angular_velocity_;}

  /**
   * @brief Get the angular velocity error
   *
   * @return Vector3& Angular velocity error (rad/s)
   */
  inline const Vector3 & get_angular_velocity_error() const
  {
    return this->get_proportional_error();
  }

protected:
  // Model
  Matrix3 inertia_ = Matrix3::Zero();               // Inertia matrix (kg m^2)
  Matrix4 mixer_matrix_inverse_ = Matrix4::Zero();  // [Fxyz, Mxyz]

  // Internal variables
  Vector3 desired_thrust_ = Vector3::Zero();          // N
  Vector3 desired_torque_ = Vector3::Zero();          // N·m
  VectorN motor_angular_velocity_ = VectorN::Zero();  // rad^2/s^2
};                                                    // Class IndiController

}  // namespace indi_controller

#endif  // AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__INDI_CONTROL__INDICONTROLLER_HPP_
