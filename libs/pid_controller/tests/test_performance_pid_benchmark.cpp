#include <benchmark/benchmark.h>
#include <exception>

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

// PID gains
Matrix3d Kp_lin_mat_ = Matrix3d::Identity();
Matrix3d Ki_lin_mat_ = Matrix3d::Identity();
Matrix3d Kd_lin_mat_ = Matrix3d::Identity();

// PID params
Vector3d antiwindup_cte_  = Vector3d::Zero();
Matrix3d alpha_           = Matrix3d::Zero();
bool reset_integral_flag_ = false;

// PID state
bool first_run_                  = true;
Vector3d integral_accum_error_   = Vector3d::Zero();
Vector3d last_proportional_error = Vector3d::Zero();
Vector3d filtered_derivate_error = Vector3d::Zero();

Vector3d limitOutput(const Vector3d &output,
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

Vector3d computeIntegral(const double &_dt, const Vector3d &_proportional_error) {
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
  integral_accum_error_ = limitOutput(integral_accum_error_, antiwindup_cte_, false);

  // Compute de integral contribution
  Vector3d i_position_error_contribution = Ki_lin_mat_ * integral_accum_error_;
  return i_position_error_contribution;
}

Vector3d computeDerivative(const double &_dt, const Vector3d &_proportional_error) {
  // Compute the derivative contribution of the error filtered with a first
  // order filter
  Vector3d proportional_error_increment = (_proportional_error - last_proportional_error);

  filtered_derivate_error = alpha_ * proportional_error_increment +
                            (Matrix3d::Identity() - alpha_) * filtered_derivate_error;

  // Compute the derivate contribution
  Vector3d derivate_error_contribution = Kd_lin_mat_ * filtered_derivate_error / _dt;
  return derivate_error_contribution;
}

Vector3d computeControl(const double &_dt, const Vector3d &_state, const Vector3d &_reference) {
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

Vector3d computeControl2(const double &_dt, const Vector3d &_state, const Vector3d &_reference) {
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
  integral_accum_error_ += proportional_error * _dt;

  // Compute anti-windup. Limit integral contribution
  integral_accum_error_ = limitOutput(integral_accum_error_, antiwindup_cte_, false);

  // Compute de integral contribution
  Vector3d integral_error_contribution = Ki_lin_mat_ * integral_accum_error_;

  // // Compute the derivate contribution
  // Compute the derivative contribution of the error filtered with a first
  // order filter
  Vector3d proportional_error_increment = (proportional_error - last_proportional_error);

  filtered_derivate_error = alpha_ * proportional_error_increment +
                            (Matrix3d::Identity() - alpha_) * filtered_derivate_error;

  // Compute the derivate contribution
  Vector3d derivate_error_contribution = Kd_lin_mat_ * filtered_derivate_error / _dt;

  // Compute output speed
  return proportional_error + integral_error_contribution + derivate_error_contribution;
}

static void BM_TEST(benchmark::State &state) {
  // PID gains
  Matrix3d Kp_lin_mat_ = Matrix3d::Identity();
  Matrix3d Ki_lin_mat_ = Matrix3d::Identity();
  Matrix3d Kd_lin_mat_ = Matrix3d::Identity();

  // PID params
  Vector3d antiwindup_cte_  = Vector3d::Zero();
  Matrix3d alpha_           = Matrix3d::Zero();
  bool reset_integral_flag_ = false;

  // PID state
  bool first_run_                  = true;
  Vector3d integral_accum_error_   = Vector3d::Zero();
  Vector3d last_proportional_error = Vector3d::Zero();
  Vector3d filtered_derivate_error = Vector3d::Zero();

  double dt          = 0.01;
  Vector3d state_vec = Vector3d::Ones();
  Vector3d ref_vec   = 2.0 * Vector3d::Ones();

  computeControl(dt, state_vec, ref_vec);
  for (auto _ : state) {
    computeControl(dt, state_vec, ref_vec);
  }
}
BENCHMARK(BM_TEST)->Threads(1)->Repetitions(10);

static void BM_TEST_PID2(benchmark::State &state) {
  // PID gains
  Matrix3d Kp_lin_mat_ = Matrix3d::Identity();
  Matrix3d Ki_lin_mat_ = Matrix3d::Identity();
  Matrix3d Kd_lin_mat_ = Matrix3d::Identity();

  // PID params
  Vector3d antiwindup_cte_  = Vector3d::Zero();
  Matrix3d alpha_           = Matrix3d::Zero();
  bool reset_integral_flag_ = false;

  // PID state
  bool first_run_                  = true;
  Vector3d integral_accum_error_   = Vector3d::Zero();
  Vector3d last_proportional_error = Vector3d::Zero();
  Vector3d filtered_derivate_error = Vector3d::Zero();

  double dt          = 0.01;
  Vector3d state_vec = Vector3d::Ones();
  Vector3d ref_vec   = 2.0 * Vector3d::Ones();

  computeControl2(dt, state_vec, ref_vec);
  for (auto _ : state) {
    computeControl2(dt, state_vec, ref_vec);
  }
}
BENCHMARK(BM_TEST_PID2)->Threads(1)->Repetitions(10);

int main(int argc, char **argv) {
  // benchmark::RegisterBenchmark("run Efficiciency", BM_TEST);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
}