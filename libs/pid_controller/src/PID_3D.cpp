#include "PID_3D.hpp"

using namespace pid_controller;

PIDController3D::PIDController3D(const bool &verbose) { _verbose = verbose; };

PIDController3D::~PIDController3D() {}

void PIDController3D::setOutputSaturation(Vector3d _saturation) {
  if (_saturation.x() == 0.0 || _saturation.y() == 0.0 || _saturation.z() == 0.0) {
    if (_verbose) {
      std::cout << "- PID-Error: Saturation must be greater than zero" << std::endl;
    }
    saturation_flag_ = false;
    return;
  }

  if (_saturation.x() < 0.0) {
    _saturation.x() = -_saturation.x();
  }
  if (_saturation.y() < 0.0) {
    _saturation.y() = -_saturation.y();
  }
  if (_saturation.z() < 0.0) {
    _saturation.z() = -_saturation.z();
  }

  output_saturation_ = _saturation;
  saturation_flag_   = true;
  return;
};

Vector3d PIDController3D::saturateOutput(const Vector3d &output,
                                         const Vector3d &limits,
                                         const bool &proportional_limitation) {
  Vector3d limited_output = output;

  // Delimit the speed for each axis
  if (proportional_limitation) {
    for (short j = 0; j < 3; j++) {
      if (limits[j] == 0.0f || output[j] == 0.0) {
        continue;
      };

      if (limited_output[j] > limits[j] || limited_output[j] < -limits[j]) {
        limited_output *= std::abs(limits[j] / limited_output[j]);
      }
    }
  } else {
    for (short j = 0; j < 3; j++) {
      if (limits[j] == 0.0f) {
        continue;
      }
      limited_output[j] = (limited_output[j] > limits[j]) ? limits[j] : limited_output[j];
      limited_output[j] = (limited_output[j] < -limits[j]) ? -limits[j] : limited_output[j];
    }
  }
  return limited_output;
}

Vector3d PIDController3D::computeIntegral(const double &_dt, const Vector3d &_proportional_error) {
  // If sing of the error changes and the integrator is saturated, reset the integral for each
  // axis
  if (reset_integral_flag_ != 0) {
    for (short j = 0; j < 3; j++) {
      if (std::abs(integral_accum_error_[j]) > antiwindup_cte_[j]) {
        if (std::signbit(integral_accum_error_[j]) != std::signbit(_proportional_error[j])) {
          integral_accum_error_[j] = 0.0f;
        }
      }
    }
  }

  // Update de acumulated error
  integral_accum_error_ += _proportional_error * _dt;

  // Compute anti-windup. Limit integral contribution
  if (antiwindup_cte_ != Vector3d::Zero()) {
    integral_accum_error_ = saturateOutput(integral_accum_error_, antiwindup_cte_, false);
  }

  // Compute de integral contribution
  Vector3d i_position_error_contribution = Ki_lin_mat_ * integral_accum_error_;
  return i_position_error_contribution;
}

Vector3d PIDController3D::computeDerivative(const double &_dt,
                                            const Vector3d &_proportional_error) {
  // Compute the derivative contribution of the error filtered with a first
  // order filter
  Vector3d proportional_error_increment = (_proportional_error - last_proportional_error);

  filtered_derivate_error = alpha_.cwiseProduct(proportional_error_increment) +
                            (Vector3d::Ones() - alpha_).cwiseProduct(filtered_derivate_error);

  // Compute the derivate contribution
  Vector3d derivate_error_contribution = Kd_lin_mat_ * filtered_derivate_error / _dt;
  return derivate_error_contribution;
}

Vector3d PIDController3D::computeDerivative(const double &_dt,
                                            const Vector3d &_state_dot,
                                            const Vector3d &_reference_dot) {
  // Get the derivate error
  Vector3d derivate_error = _state_dot - _reference_dot;

  // Compute the derivate contribution
  Vector3d derivate_error_contribution = Kd_lin_mat_ * derivate_error / _dt;
  return derivate_error_contribution;
}

Vector3d PIDController3D::computeControl(const double &_dt,
                                         const Vector3d &_state,
                                         const Vector3d &_reference) {
  // Get the error
  Vector3d proportional_error = _reference - _state;

  // Initialize values for the integral and derivative contributions
  if (first_run_) {
    first_run_              = false;
    integral_accum_error_   = Vector3d::Zero();
    last_proportional_error = proportional_error;
    filtered_derivate_error = Vector3d::Zero();
  }

  // Compute the proportional contribution
  Vector3d p_error_contribution = Kp_lin_mat_ * proportional_error;

  // // Compute de integral contribution (position integrate)
  Vector3d integral_error_contribution = computeIntegral(_dt, proportional_error);

  // // Compute the derivate contribution
  Vector3d derivate_error_contribution = computeDerivative(_dt, proportional_error);

  // Compute output speed
  return proportional_error + integral_error_contribution + derivate_error_contribution;
}

Vector3d PIDController3D::computeControlWithSaturation(const double &_dt,
                                                       const Vector3d &_state,
                                                       const Vector3d &_reference) {
  // Compute the control
  Vector3d output = computeControl(_dt, _state, _reference);

  if (saturation_flag_) {
    return saturateOutput(output, output_saturation_, proportional_saturation_);
  }
  return output;
}

Vector3d PIDController3D::computeControl(const double &_dt,
                                         const Vector3d &_state,
                                         const Vector3d &_reference,
                                         const Vector3d &_state_dot,
                                         const Vector3d &_reference_dot) {
  // Get the error
  Vector3d proportional_error = _reference - _state;

  // Initialize values for the integral and derivative contributions
  if (first_run_) {
    first_run_              = false;
    integral_accum_error_   = Vector3d::Zero();
    last_proportional_error = proportional_error;
    filtered_derivate_error = Vector3d::Zero();
  }

  // Compute the proportional contribution
  Vector3d p_error_contribution = Kp_lin_mat_ * proportional_error;

  // // Compute de integral contribution (position integrate)
  Vector3d integral_error_contribution = computeIntegral(_dt, proportional_error);

  // // Compute the derivate contribution
  Vector3d derivate_error_contribution = computeDerivative(_dt, _state_dot, _reference_dot);

  // Compute output speed
  return proportional_error + integral_error_contribution + derivate_error_contribution;
}

Vector3d PIDController3D::computeControl(const Vector3d &_state,
                                         const Vector3d &_reference,
                                         const Vector3d &_state_dot,
                                         const Vector3d &_reference_dot) {
  // Get the error
  Vector3d proportional_error = _reference - _state;

  // Initialize values for the integral and derivative contributions
  if (first_run_) {
    first_run_              = false;
    integral_accum_error_   = Vector3d::Zero();
    last_proportional_error = proportional_error;
    filtered_derivate_error = Vector3d::Zero();
  }

  // Compute the proportional contribution
  Vector3d p_error_contribution = Kp_lin_mat_ * proportional_error;

  // // Compute de integral contribution (position integrate)
  // If sing of the error changes and the integrator is saturated, reset the integral for each
  // axis
  if (reset_integral_flag_ != 0) {
    for (short j = 0; j < 3; j++) {
      if (std::abs(integral_accum_error_[j]) > antiwindup_cte_[j]) {
        if (std::signbit(integral_accum_error_[j]) != std::signbit(proportional_error[j])) {
          integral_accum_error_[j] = 0.0f;
        }
      }
    }
  }

  // Update de acumulated error
  integral_accum_error_ += proportional_error;

  // Compute anti-windup. Limit integral contribution
  if (antiwindup_cte_ != Vector3d::Zero()) {
    integral_accum_error_ = saturateOutput(integral_accum_error_, antiwindup_cte_, false);
  }

  // Compute de integral contribution
  Vector3d integral_error_contribution = Ki_lin_mat_ * integral_accum_error_;
  // Vector3d integral_error_contribution = computeIntegral(_dt, proportional_error);

  // // Compute the derivate contribution
  // Get the derivate error
  Vector3d derivate_error = _state_dot - _reference_dot;

  // Compute the derivate contribution
  Vector3d derivate_error_contribution = Kd_lin_mat_ * derivate_error;
  // Vector3d derivate_error_contribution = computeDerivative(_dt, _state_dot, _reference_dot);

  // Compute output speed
  return proportional_error + integral_error_contribution + derivate_error_contribution;
}

Vector3d PIDController3D::computeControlWithSaturation(const double &_dt,
                                                       const Vector3d &_state,
                                                       const Vector3d &_reference,
                                                       const Vector3d &_state_dot,
                                                       const Vector3d &_reference_dot) {
  // Compute the control
  Vector3d output = computeControl(_dt, _state, _reference, _state_dot, _reference_dot);

  if (saturation_flag_) {
    return saturateOutput(output, output_saturation_, proportional_saturation_);
  }
  return output;
}
