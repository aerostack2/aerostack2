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
 *  \file       pid.hpp
 *  \brief      PID Controller definition
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

#ifndef AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__INDI_CONTROL_IGN__PIDCONTROLLER_HPP_
#define AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__INDI_CONTROL_IGN__PIDCONTROLLER_HPP_

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <limits>

namespace pid_controller
{

template<typename P = double, int dim = 3>
struct PIDParams
{
  static_assert(
    std::is_floating_point<P>::value,
    "MotorParams must be used with a floating-point type");
  static_assert(dim > 0, "MotorParams must be used with a positive dimension");

  using Vector = Eigen::Matrix<P, dim, 1>;
  using Matrix = Eigen::Matrix<P, dim, dim>;

  // PID gains
  Vector Kp_gains = Vector::Zero();  // Proportional gains
  Vector Ki_gains = Vector::Zero();  // Integral gains
  Vector Kd_gains = Vector::Zero();  // Derivative gains

  // PID params
  Vector antiwindup_cte = Vector::Zero();     // Integral anti-windup
  Vector alpha = Vector::Ones();              // Derivative filter
  bool reset_integral_flag = false;           // Reset integral flag when error sign changes

  // PID Output saturation
  bool proportional_saturation_flag = false;           // Output proportional saturation flag
  Vector upper_output_saturation = Vector::Zero();     // Upper output saturation
  Vector lower_output_saturation = Vector::Zero();     // Lower output saturation
};

/**
 * @brief PID controller
 *
 * @tparam P Precision
 * @tparam dim Dimension
 */
template<typename P = double, int dim = 3>
class PID
{
  static_assert(
    std::is_floating_point<P>::value,
    "MotorParams must be used with a floating-point type");
  static_assert(dim > 0, "MotorParams must be used with a positive dimension");

  using Scalar = P;
  using Vector = Eigen::Matrix<P, dim, 1>;
  using Matrix = Eigen::Matrix<P, dim, dim>;

public:
  /**
   * @brief Construct a new PID
   *
   * @param verbose Verbosity flag. Default: false
   */
  explicit PID(
    const PIDParams<P, dim> & pid_params = PIDParams<P, dim>(),
    const bool & verbose = false)
  : verbose_(verbose)
  {
    update_pid_params(pid_params);
    reset_controller();
  }

  ~PID() {}

protected:
  bool verbose_ = false;  // Verbosity flag

  // PID gains
  Matrix Kp_lin_mat_ = Matrix::Identity();
  Matrix Ki_lin_mat_ = Matrix::Identity();
  Matrix Kd_lin_mat_ = Matrix::Identity();

  // PID params
  Vector antiwindup_cte_ = Vector::Zero();     // Integral anti-windup
  Vector alpha_ = Vector::Zero();              // Derivative filter
  bool reset_integral_flag_ = false;           // Reset integral flag when error sign changes

  // PID Output saturation
  bool saturation_flag_ = false;                        // Output saturation flag
  bool proportional_saturation_flag_ = false;           // Output proportional saturation flag
  Vector upper_output_saturation_ = Vector::Zero();     // Upper output saturation
  Vector lower_output_saturation_ = Vector::Zero();     // Lower output saturation

  // PID state
  bool first_run_ = true;                            // First run flag
  Vector integral_accum_error_ = Vector::Zero();     // Integral accumulator error
  Vector filtered_derivate_error_ = Vector::Zero();  // Filtered derivative error

  // Error and output storage (for debugging purposes)
  Vector proportional_error_ = Vector::Zero();               // Proportional error
  Vector derivative_error_ = Vector::Zero();                 // Derivative error
  Vector proportional_error_contribution_ = Vector::Zero();  // Proportional error contribution
  Vector integral_error_contribution_ = Vector::Zero();      // Integral error contribution
  Vector derivate_error_contribution_ = Vector::Zero();      // Derivative error contribution
  Vector output_ = Vector::Zero();                           // Output

public:
  // Public methods

  /**
   * @brief Update the PID controller with pid params
   *
   * @param params PIDParams struct
   */
  void update_pid_params(const PIDParams<P, dim> & params)
  {
    set_gains(params.Kp_gains, params.Ki_gains, params.Kd_gains);
    set_anti_windup(params.antiwindup_cte);
    set_alpha(params.alpha);
    set_reset_integral_saturation_flag(params.reset_integral_flag);
    set_proportional_saturation_flag(params.proportional_saturation_flag);

    if (params.lower_output_saturation != Eigen::Matrix<P, dim, 1>::Zero() ||
      params.upper_output_saturation != Eigen::Matrix<P, dim, 1>::Zero())
    {
      set_output_saturation(
        params.upper_output_saturation, params.lower_output_saturation,
        params.proportional_saturation_flag);
    } else {
      disable_output_saturation();
    }
  }

  /**
   * @brief Reset the controller
   *
   * Reset the integral error
   */
  inline void reset_controller() {first_run_ = true;}

  /**
   * @brief Set the output saturation
   *
   * @param upper_saturation Upper saturation
   * @param lower_saturation Lower saturation
   * @param proportional_saturation_flag Proportional saturation flag. Default: false
   */
  void set_output_saturation(
    const Vector & upper_saturation,
    const Vector & lower_saturation,
    bool proportional_saturation_flag = false)
  {
    for (int i = 0; i < dim; i++) {
      // Check if different between upper and lower saturation is greater than epsilon
      if (std::abs(upper_saturation[i] - lower_saturation[i]) <
        std::numeric_limits<Scalar>::epsilon())
      {
        std::cerr << "Upper and lower saturation are equal. Saturation is disabled" << std::endl;
        disable_output_saturation();
        return;
      }
      // Check if upper saturation is greater than lower saturation
      if (upper_saturation[i] < lower_saturation[i]) {
        std::cerr << "Upper saturation is lower than lower saturation. Saturation is disabled"
                  << std::endl;
        disable_output_saturation();
        return;
      }
      upper_output_saturation_ = upper_saturation;
      lower_output_saturation_ = lower_saturation;
    }
    saturation_flag_ = true;
    proportional_saturation_flag_ = proportional_saturation_flag;
    return;
  }

  /**
   * @brief Disable the output saturation
   *
   * Disable the output saturation. The output is not limited by the saturation limits.
   * To enable the output saturation, use the set_output_saturation method.
   *
   * @param saturation_flag Saturation flag
   */
  inline void disable_output_saturation() {saturation_flag_ = false;}

  /**
   * @brief Get the proportional error
   *
   * @param state Current state
   * @param reference Reference state
   * @return Vector Error
   */
  static inline Vector get_error(const Vector & state, const Vector & reference)
  {
    // Compute proportional error
    return reference - state;
  }

  /**
   * @brief Get the proportional and derivative error
   *
   * @param state State
   * @param reference Reference
   * @param state_dot State derivative
   * @param reference_dot Reference derivative
   * @param proportional_error Output proportional error
   * @param derivative_error Output derivative error
   */
  static inline void get_error(
    const Vector & state,
    const Vector & reference,
    const Vector & state_dot,
    const Vector & reference_dot,
    Vector & proportional_error,
    Vector & derivative_error)
  {
    // Compute proportional error
    proportional_error = reference - state;

    // Compute the derivate error
    derivative_error = reference_dot - state_dot;
  }

  /**
   * @brief Process the PID controller
   *
   * @param dt Time step
   * @param proportional_error Proportional error
   * @return Vector PID output
   */
  Vector compute_control(const Scalar dt, const Vector & proportional_error)
  {
    // Initialize values for the integral and derivative contributions
    if (first_run_) {
      first_run_ = false;
      integral_accum_error_ = Vector::Zero();
      proportional_error_ = proportional_error;
      filtered_derivate_error_ = Vector::Zero();
    }

    // Compute the proportional contribution
    proportional_error_contribution_ = Kp_lin_mat_ * proportional_error;

    // Compute de integral contribution
    integral_error_contribution_ = compute_integral_contribution(dt, proportional_error);

    // Compute the derivate contribution
    derivate_error_contribution_ =
      compute_derivative_contribution_by_deriving(dt, proportional_error);

    // Compute output
    output_ = proportional_error_contribution_ + integral_error_contribution_ +
      derivate_error_contribution_;

    // Saturation
    if (saturation_flag_) {
      output_ = saturate_output(
        output_, upper_output_saturation_, lower_output_saturation_,
        proportional_saturation_flag_);
    }

    // Update last proportional error
    proportional_error_ = proportional_error;

    return output_;
  }

  /**
   * @brief Process the PID controller with derivative feedback
   *
   * Derivative feedback is used to improve the controller performance.
   *
   * @param dt Time step (s)
   * @param state Current state
   * @param reference Reference state
   * @param state_dot Current state derivative
   * @param reference_dot Reference state derivative
   * @return Vector
   */
  Vector compute_control(
    const Scalar dt,
    const Vector & proportional_error,
    const Vector & derivative_error)
  {
    // Initialize values for the integral and derivative contributions
    if (first_run_) {
      first_run_ = false;
      integral_accum_error_ = Vector::Zero();
      proportional_error_ = proportional_error;
      filtered_derivate_error_ = Vector::Zero();
    }

    // Compute the proportional contribution
    proportional_error_contribution_ = Kp_lin_mat_ * proportional_error;

    // Compute de integral contribution
    integral_error_contribution_ = compute_integral_contribution(dt, proportional_error);

    // Compute the derivate contribution
    derivate_error_contribution_ = compute_derivative_contribution(derivative_error);

    // Compute output
    output_ = proportional_error_contribution_ + integral_error_contribution_ +
      derivate_error_contribution_;

    // Saturation
    if (saturation_flag_) {
      output_ = saturate_output(
        output_, upper_output_saturation_, lower_output_saturation_,
        proportional_saturation_flag_);
    }

    // Update last error
    proportional_error_ = proportional_error;
    derivative_error_ = derivative_error;

    return output_;
  }

  /**
   * @brief Saturation function
   *
   * If the output is greater than the upper limit, the output is saturated to the upper limit.
   * If the output is lower than the lower limit, the output is saturated to the lower limit.
   * If proportional_saturation is true, the output is saturated proportionally to the limits,
   * keeping the vector direction.
   *
   * @param output Vector to saturate
   * @param upper_limits Upper limits vector
   * @param lower_limits Lower limits vector
   * @param proportional_saturation Proportional saturation flag. Default: false
   * @return Vector Saturated vector
   */
  static Vector saturate_output(
    const Vector & output,
    const Vector & upper_limits,
    const Vector & lower_limits,
    const bool proportional_saturation = false)
  {
    Vector saturated_output = output;

    // Non proportional saturation
    if (!proportional_saturation) {
      for (int j = 0; j < output.size(); j++) {
        if (output[j] > upper_limits[j]) {
          saturated_output[j] = upper_limits[j];
        } else if (output[j] < lower_limits[j]) {
          saturated_output[j] = lower_limits[j];
        }
      }
      return saturated_output;
    }

    // Proportional saturation
    Scalar factor = 1.0;
    for (int j = 0; j < saturated_output.size(); j++) {
      factor = 1.0;
      if (saturated_output[j] > upper_limits[j]) {
        factor = upper_limits[j] / saturated_output[j];
      } else if (saturated_output[j] < lower_limits[j]) {
        factor = lower_limits[j] / saturated_output[j];
      }
      saturated_output = factor * saturated_output;
    }

    return saturated_output;
  }
  // Getters and setters

  /**
   * @brief Get the params
   *
   * @return PIDParams<P, dim> PID parameters
   */
  PIDParams<P, dim> get_params() const
  {
    PIDParams<P, dim> params;
    get_gains(params.Kp_gains, params.Ki_gains, params.Kd_gains);
    params.antiwindup_cte = get_anti_windup();
    params.alpha = get_alpha();
    params.reset_integral_flag = get_reset_integral_saturation_flag();
    params.proportional_saturation_flag = get_proportional_saturation_flag();
    get_saturation_limits(params.upper_output_saturation, params.lower_output_saturation);
    return params;
  }

  /**
   * @brief Set the Gains of the controller
   *
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  inline void set_gains(const Vector & kp, const Vector & ki, const Vector & kd)
  {
    Kp_lin_mat_ = kp.asDiagonal();
    Ki_lin_mat_ = ki.asDiagonal();
    Kd_lin_mat_ = kd.asDiagonal();
  }

  /**
   * @brief Get the gains
   *
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  inline void get_gains(Vector & kp, Vector & ki, Vector & kd) const
  {
    kp = Kp_lin_mat_.diagonal();
    ki = Ki_lin_mat_.diagonal();
    kd = Kd_lin_mat_.diagonal();
  }

  /**
   * @brief Set the gains kp
   *
   * @param kp Proportional gain
   */
  inline void set_gains_kp(const Vector & kp) {Kp_lin_mat_ = kp.asDiagonal();}

  /**
   * @brief Get the gains kp
   *
   * @param kp Proportional gain
   */
  inline Vector get_gains_kp() const {return Kp_lin_mat_.diagonal();}

  /**
   * @brief Set the gains ki
   *
   * @param ki Integral gain
   */
  inline void set_gains_ki(const Vector & ki) {Ki_lin_mat_ = ki.asDiagonal();}

  /**
   * @brief Get the gains ki
   *
   * @param ki Integral gain
   */
  inline Vector get_gains_ki() const {return Ki_lin_mat_.diagonal();}

  /**
   * @brief Set the gains kd
   *
   * @param kd Derivative gain
   */
  inline void set_gains_kd(const Vector & kd) {Kd_lin_mat_ = kd.asDiagonal();}

  /**
   * @brief Get the gains kd
   *
   * @param kd Derivative gain
   */
  inline Vector get_gains_kd() const {return Kd_lin_mat_.diagonal();}

  /**
   * @brief Set the anti windup
   *
   * Anti windup is a vector that limits the integral contribution of the controller. If the
   * integral contribution is greater/lower than the anti windup, the integral contribution is
   * set to anti windup value.
   *
   * @param anti_windup Anti windup
   */
  inline void set_anti_windup(const Vector & anti_windup) {antiwindup_cte_ = anti_windup;}

  /**
   * @brief Get the anti windup
   *
   * Anti windup is a value that limits the integral contribution of the controller. If the
   * integral contribution is greater/lower than the anti windup, the integral contribution is
   * set to anti windup value.
   *
   * @param anti_windup Anti windup
   */
  inline Vector get_anti_windup() const {return antiwindup_cte_;}

  /**
   * @brief Set the alpha
   *
   * Alpha is a value that filters the derivative contribution of the controller. If alpha is
   * 1, the derivative contribution is not filtered.
   *
   * @param alpha Alpha in (0, 1]
   */
  inline void set_alpha(const Vector alpha) {alpha_ = alpha;}

  /**
   * @brief Get the alpha
   *
   * Alpha is a value that filters the derivative contribution of the controller. If alpha is
   * 1, the derivative contribution is not filtered.
   *
   * @param alpha Alpha in (0, 1]
   */
  inline Vector get_alpha() const {return alpha_;}

  /**
   * @brief Set the reset integral saturation flag
   *
   * If the flag is true, the integral contribution is reset to zero when the integral error
   * is grater than the anti windup and the sign of the integral error is different from the
   * sign of the proportional error.
   *
   * @param reset_integral_flag Reset integral saturation flag
   */
  inline void set_reset_integral_saturation_flag(bool reset_integral_flag)
  {
    reset_integral_flag_ = reset_integral_flag;
  }

  /**
   * @brief Get the reset integral saturation flag
   *
   * If the flag is true, the integral contribution is reset to zero when the integral error
   * is grater than the anti windup and the sign of the integral error is different from the
   * sign of the proportional error.
   *
   * @param reset_integral_flag Reset integral saturation flag
   */
  inline bool get_reset_integral_saturation_flag() const {return reset_integral_flag_;}

  /**
   * @brief Set the proportional saturation flag
   *
   * If the flag is true, the output is saturated proportionally to the saturation limits.
   * So, the output vector keeps the same direction but its norm is limited to the saturation
   *
   * @param proportional_saturation_flag Proportional saturation flag
   */
  inline void set_proportional_saturation_flag(bool proportional_saturation_flag)
  {
    proportional_saturation_flag_ = proportional_saturation_flag;
  }

  /**
   * @brief Get the proportional saturation flag
   *
   * If the flag is true, the output is saturated proportionally to the saturation limits.
   * So, the output vector keeps the same direction but its norm is limited to the saturation
   *
   * @param proportional_saturation_flag Proportional saturation flag
   */
  inline bool get_proportional_saturation_flag() const {return proportional_saturation_flag_;}

  /**
   * @brief Get the saturation limits
   *
   * @param saturation_limits Saturation limits
   */
  inline void get_saturation_limits(
    Vector & saturation_upper_limit,
    Vector & saturation_lower_limit) const
  {
    saturation_upper_limit = upper_output_saturation_;
    saturation_lower_limit = lower_output_saturation_;
  }

  /**
   * @brief Get the output saturation flag
   *
   * @return true Saturation is enabled
   * @return false Saturation is disabled
   */
  inline bool get_output_saturation_flag() const {return saturation_flag_;}

  /**
   * @brief Get the proportional error
   *
   * @return Vector Proportional error
   */
  inline const Vector & get_proportional_error() const {return proportional_error_;}

  /**
   * @brief Get the derivative error
   *
   * @return Vector Derivative error
   */
  inline const Vector & get_derivative_error() const {return derivative_error_;}

  /**
   * @brief Get the proportional error contribution
   *
   * @return Vector Proportional error contribution
   */
  inline const Vector & get_proportional_error_contribution() const
  {
    return proportional_error_contribution_;
  }

  /**
   * @brief Get the integral error contribution
   *
   * @return Vector Integral error contribution
   */
  inline const Vector & get_integral_error_contribution() const
  {
    return integral_error_contribution_;
  }

  /**
   * @brief Get the derivative error contribution
   *
   * @return Vector Derivative error contribution
   */
  inline const Vector & get_derivative_error_contribution() const
  {
    return derivate_error_contribution_;
  }

  /**
   * @brief Get the output
   *
   * @return Vector Output
   */
  inline const Vector & get_output() const {return output_;}

protected:
  /**
   * @brief Compute the integral contribution of the controller
   *
   * @param dt Delta time (s)
   * @param proportional_error Proportional error
   * @return Vector Integral contribution
   */
  Vector compute_integral_contribution(const Scalar dt, const Vector & proportional_error)
  {
    // If sing of the error changes and the integrator is saturated, reset the integral for each
    // axis
    if (reset_integral_flag_ != 0) {
      for (int j = 0; j < proportional_error.size(); j++) {
        if (std::abs(integral_accum_error_[j]) > antiwindup_cte_[j]) {
          if (std::signbit(integral_accum_error_[j]) != std::signbit(proportional_error[j])) {
            integral_accum_error_[j] = 0.0;
          }
        }
      }
    }

    // Update de acumulated error
    integral_accum_error_ += proportional_error * dt;

    // Compute anti-windup. Limit integral contribution
    if (antiwindup_cte_ != Vector::Zero()) {
      integral_accum_error_ =
        saturate_output(integral_accum_error_, antiwindup_cte_, -1.0 * antiwindup_cte_, false);
    }

    // Compute de integral contribution
    Vector integral_error_contribution = Ki_lin_mat_ * integral_accum_error_;
    return integral_error_contribution;
  }

  /**
   * @brief Compute the derivative contribution of the controller
   *
   * @param dt Delta time (s)
   * @param proportional_error Proportional error
   * @return Vector Derivative contribution
   */
  Vector compute_derivative_contribution_by_deriving(
    const Scalar dt,
    const Vector & proportional_error)
  {
    // Compute the derivative contribution of the error filtered with a first
    // order filter
    Vector derivate_proportional_error_increment = (proportional_error - proportional_error_) / dt;

    filtered_derivate_error_ = alpha_.cwiseProduct(derivate_proportional_error_increment) +
      (Vector::Ones() - alpha_).cwiseProduct(filtered_derivate_error_);

    // Compute the derivate contribution
    Vector derivate_error_contribution = compute_derivative_contribution(filtered_derivate_error_);
    return derivate_error_contribution;
  }

  /**
   * @brief Compute the derivative contribution of the controller
   *
   * For controllers with derivative feedback, the derivative contribution is computed using the
   * state and reference derivatives.
   *
   * @param state_dot State derivative
   * @param reference_dot Reference derivative
   * @return Vector Derivative contribution
   */
  inline Vector compute_derivative_contribution(const Vector & derivate_error)
  {
    // Compute the derivate contribution
    Vector derivate_error_contribution = Kd_lin_mat_ * derivate_error;
    return derivate_error_contribution;
  }
};

}  // namespace pid_controller

#endif  // AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__INDI_CONTROL_IGN__PIDCONTROLLER_HPP_
