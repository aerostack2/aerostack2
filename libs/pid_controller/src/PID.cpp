#include "PID.hpp"

using namespace pid_controller;

PIDController::PIDController(const bool &verbose) { _verbose = verbose; };

PIDController::~PIDController() {}

void PIDController::setOutputSaturation(double saturation) {
  if ((saturation == 0.0)) {
    if (_verbose) {
      std::cout << "- PID-Error: Saturation must be greater than zero" << std::endl;
    }
    return;
  }

  if (saturation < 0.0) {
    saturation = -saturation;
  }
  output_max_saturation_ = saturation;
  output_min_saturation_ = -saturation;
  saturation_flag_       = true;
};

void PIDController::setOutputSaturation(double _min_saturation, double _max_saturation) {
  if (_max_saturation > _min_saturation) {
    output_max_saturation_ = _max_saturation;
    output_min_saturation_ = _min_saturation;
    saturation_flag_       = true;
  } else if (_verbose) {
    std::cout << "- PID-Error: Max saturation must be greater than min saturation" << std::endl;
  }
};

double PIDController::saturateOutput(const double &output, const double &limit) {
  double limited_output = output;

  if (limit != 0.0f) {
    limited_output = (limited_output > limit) ? limit : limited_output;
    limited_output = (limited_output < -limit) ? -limit : limited_output;
  }
  return limited_output;
}

double PIDController::saturateOutput(const double &output,
                                     const double &min_limit,
                                     const double &max_limit) {
  double limited_output = output;
  limited_output        = (limited_output > max_limit) ? max_limit : limited_output;
  limited_output        = (limited_output < min_limit) ? min_limit : limited_output;
  return limited_output;
}

double PIDController::computeIntegral(const double &_dt, const double &_proportional_error) {
  // If sing of the error changes and the integrator is saturated, reset the integral for each
  // axis
  if (reset_integral_flag_ != 0) {
    if (std::abs(integral_accum_error_) > antiwindup_cte_) {
      if (std::signbit(integral_accum_error_) != std::signbit(_proportional_error)) {
        integral_accum_error_ = 0.0f;
      }
    }
  }

  // Update de acumulated error
  integral_accum_error_ += _proportional_error * _dt;

  // Compute anti-windup. Limit integral contribution
  if (antiwindup_cte_ != 0.0f) {
    integral_accum_error_ = saturateOutput(integral_accum_error_, antiwindup_cte_);
  }

  // Compute de integral contribution
  double i_position_error_contribution = Ki_ * integral_accum_error_;
  return i_position_error_contribution;
}

double PIDController::computeDerivative(const double &_dt, const double &_proportional_error) {
  // Compute the derivative contribution of the error filtered with a first
  // order filter
  double proportional_error_increment = (_proportional_error - last_proportional_error);

  filtered_derivate_error =
      alpha_ * proportional_error_increment + (1.0 - alpha_) * filtered_derivate_error;

  // Compute the derivate contribution
  double derivate_error_contribution = Kd_ * filtered_derivate_error / _dt;
  return derivate_error_contribution;
}

double PIDController::computeDerivative(const double &_dt,
                                        const double &_state_dot,
                                        const double &_reference_dot) {
  // Get the derivate error
  double derivate_error = _state_dot - _reference_dot;

  // Compute the derivate contribution
  double derivate_error_contribution = Kd_ * derivate_error / _dt;
  return derivate_error_contribution;
}

double PIDController::computeControl(const double &_dt, const double &proportional_error) {
  // Initialize values for the integral and derivative contributions
  if (first_run_) {
    first_run_              = false;
    integral_accum_error_   = 0.0;
    last_proportional_error = proportional_error;
    filtered_derivate_error = 0.0;
  }

  // Compute the proportional contribution
  double p_error_contribution = Kp_ * proportional_error;

  // // Compute de integral contribution (position integrate)
  double integral_error_contribution = computeIntegral(_dt, proportional_error);

  // // Compute the derivate contribution
  double derivate_error_contribution = computeDerivative(_dt, proportional_error);

  // Compute output speed
  return proportional_error + integral_error_contribution + derivate_error_contribution;
}

double PIDController::computeControl(const double &_dt,
                                     const double &_state,
                                     const double &_reference) {
  // Get the error
  double proportional_error = _reference - _state;

  // Compute the control
  return computeControl(_dt, proportional_error);
}

double PIDController::computeControlWithSaturation(const double &_dt,
                                                   const double &_state,
                                                   const double &_reference) {
  // Compute control
  double output = computeControl(_dt, _state, _reference);

  // Limit output
  if (saturation_flag_) {
    return saturateOutput(output, output_min_saturation_, output_max_saturation_);
  }
  return output;
}

double PIDController::computeControl(const double &_dt,
                                     const double &_state,
                                     const double &_reference,
                                     const double &_state_dot,
                                     const double &_reference_dot) {
  // Get the error
  double proportional_error = _reference - _state;

  // Initialize values for the integral and derivative contributions
  if (first_run_) {
    first_run_              = false;
    integral_accum_error_   = 0.0;
    last_proportional_error = proportional_error;
    filtered_derivate_error = 0.0;
  }

  // Compute the proportional contribution
  double p_error_contribution = Kp_ * proportional_error;

  // // Compute de integral contribution (position integrate)
  double integral_error_contribution = computeIntegral(_dt, proportional_error);

  // // Compute the derivate contribution
  double derivate_error_contribution = computeDerivative(_dt, _state_dot, _reference_dot);

  // Compute output speed
  return proportional_error + integral_error_contribution + derivate_error_contribution;
}

double PIDController::computeControlWithSaturation(const double &_dt,
                                                   const double &_state,
                                                   const double &_reference,
                                                   const double &_state_dot,
                                                   const double &_reference_dot) {
  // Compute control
  double output = computeControl(_dt, _state, _reference, _state_dot, _reference_dot);

  // Limit output
  if (saturation_flag_) {
    return saturateOutput(output, output_min_saturation_, output_max_saturation_);
  }
  return output;
}
