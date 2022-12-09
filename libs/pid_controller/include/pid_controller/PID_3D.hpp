#ifndef PID_CONTROLLER_3D_HPP_
#define PID_CONTROLLER_3D_HPP_

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace pid_controller {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

class PIDController3D {
public:
  PIDController3D(const bool &verbose = false);
  ~PIDController3D();

  inline void setGains(Vector3d _kp, Vector3d _ki, Vector3d _kd) {
    Kp_lin_mat_ = _kp.asDiagonal();
    Ki_lin_mat_ = _ki.asDiagonal();
    Kd_lin_mat_ = _kd.asDiagonal();
  };
  inline void setGainKpX(double _kp) { Kp_lin_mat_(0, 0) = _kp; };
  inline void setGainKpY(double _kp) { Kp_lin_mat_(1, 1) = _kp; };
  inline void setGainKpZ(double _kp) { Kp_lin_mat_(2, 2) = _kp; };
  inline void setGainKiX(double _ki) { Ki_lin_mat_(0, 0) = _ki; };
  inline void setGainKiY(double _ki) { Ki_lin_mat_(1, 1) = _ki; };
  inline void setGainKiZ(double _ki) { Ki_lin_mat_(2, 2) = _ki; };
  inline void setGainKdX(double _kd) { Kd_lin_mat_(0, 0) = _kd; };
  inline void setGainKdY(double _kd) { Kd_lin_mat_(1, 1) = _kd; };
  inline void setGainKdZ(double _kd) { Kd_lin_mat_(2, 2) = _kd; };
  inline void setAntiWindup(Vector3d _anti_windup) { antiwindup_cte_ = _anti_windup; };
  inline void setAntiWindup(double _anti_windup) {
    antiwindup_cte_ = Vector3d::Constant(_anti_windup);
  };
  inline void setAlpha(Vector3d _alpha) { alpha_ = _alpha; };
  inline void setAlpha(double _alpha) { alpha_ = Vector3d::Constant(_alpha); };
  inline void setResetIntegralSaturationFlag(bool _reset_integral_flag) {
    reset_integral_flag_ = _reset_integral_flag;
  };

  void setProportionalSaturation(bool _proportional_saturation) {
    proportional_saturation_ = _proportional_saturation;
  };
  inline void disableSaturation() { saturation_flag_ = false; };
  void setOutputSaturation(Vector3d _saturation);

  inline void resetController() {
    first_run_       = true;
    saturation_flag_ = false;
  };

  Vector3d computeControl(const double &_dt, const Vector3d &_state, const Vector3d &_reference);

  Vector3d computeControlWithSaturation(const double &_dt,
                                        const Vector3d &_state,
                                        const Vector3d &_reference);

  Vector3d computeControl(const double &_dt,
                          const Vector3d &_state,
                          const Vector3d &_reference,
                          const Vector3d &_state_dot,
                          const Vector3d &_reference_dot);

  Vector3d computeControl(const Vector3d &_state,
                          const Vector3d &_reference,
                          const Vector3d &_state_dot,
                          const Vector3d &_reference_dot);

  Vector3d computeControlWithSaturation(const double &_dt,
                                        const Vector3d &_state,
                                        const Vector3d &_reference,
                                        const Vector3d &_state_dot,
                                        const Vector3d &_reference_dot);

  static Vector3d saturateOutput(const Vector3d &output,
                                 const Vector3d &limits,
                                 const bool &proportional_limitation);

private:
  Vector3d computeIntegral(const double &_dt, const Vector3d &_proportional_error);
  Vector3d computeDerivative(const double &_dt, const Vector3d &_proportional_error);
  Vector3d computeDerivative(const double &_dt,
                             const Vector3d &_state_dot,
                             const Vector3d &_reference_dot);

private:
  bool _verbose = true;

  // PID gains
  Matrix3d Kp_lin_mat_ = Matrix3d::Identity();
  Matrix3d Ki_lin_mat_ = Matrix3d::Identity();
  Matrix3d Kd_lin_mat_ = Matrix3d::Identity();

  // PID params
  Vector3d antiwindup_cte_      = Vector3d::Zero();
  Vector3d alpha_               = Vector3d::Zero();
  bool reset_integral_flag_     = false;
  Vector3d output_saturation_   = Vector3d::Zero();
  bool saturation_flag_         = false;
  bool proportional_saturation_ = false;

  // PID state
  bool first_run_                  = true;
  Vector3d integral_accum_error_   = Vector3d::Zero();
  Vector3d last_proportional_error = Vector3d::Zero();
  Vector3d filtered_derivate_error = Vector3d::Zero();
};
}  // namespace pid_controller

#endif  // PID_CONTROLLER_3D_HPP_
