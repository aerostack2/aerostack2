#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace pid_controller {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

class PIDController {
public:
  PIDController(const bool &verbose = false);
  ~PIDController();

  inline void setGains(double _kp, double _ki, double _kd) {
    Kp_ = _kp;
    Ki_ = _ki;
    Kd_ = _kd;
  };
  inline void setGainKp(double _kp) { Kp_ = _kp; };
  inline void setGainKi(double _ki) { Ki_ = _ki; };
  inline void setGainKd(double _kd) { Kd_ = _kd; };
  inline void setAntiWindup(double _anti_windup) { antiwindup_cte_ = _anti_windup; };
  inline void setAlpha(double _alpha) { alpha_ = _alpha; };
  inline void setResetIntegralSaturationFlag(bool _reset_integral_flag) {
    reset_integral_flag_ = _reset_integral_flag;
  };

  inline void disableSaturation() { saturation_flag_ = false; };
  void setOutputSaturation(double saturation);
  void setOutputSaturation(double _min_saturation, double _max_saturation);
  static double saturateOutput(const double &output, const double &limit);
  static double saturateOutput(const double &output,
                               const double &min_limit,
                               const double &max_limit);

  inline void resetController() {
    first_run_       = true;
    saturation_flag_ = false;
  };

  double computeControl(const double &_dt, const double &_proportional_error);

  double computeControl(const double &_dt, const double &_state, const double &_reference);

  double computeControlWithSaturation(const double &_dt,
                                      const double &_state,
                                      const double &_reference);

  double computeControl(const double &_dt,
                        const double &_state,
                        const double &_reference,
                        const double &_state_dot,
                        const double &_reference_dot);

  double computeControlWithSaturation(const double &_dt,
                                      const double &_state,
                                      const double &_reference,
                                      const double &_state_dot,
                                      const double &_reference_dot);

private:
  double computeIntegral(const double &_dt, const double &_proportional_error);
  double computeDerivative(const double &_dt, const double &_proportional_error);
  double computeDerivative(const double &_dt,
                           const double &_state_dot,
                           const double &_reference_dot);

private:
  bool _verbose = true;

  // PID gains
  double Kp_ = 0.0;
  double Ki_ = 0.0;
  double Kd_ = 0.0;

  // PID params
  double antiwindup_cte_        = 0.0;
  double alpha_                 = 0.0;
  bool reset_integral_flag_     = false;
  bool saturation_flag_         = false;
  double output_max_saturation_ = 0.0;
  double output_min_saturation_ = 0.0;

  // PID state
  bool first_run_                = true;
  double integral_accum_error_   = 0.0;
  double last_proportional_error = 0.0;
  double filtered_derivate_error = 0.0;
};
}  // namespace pid_controller

#endif  // PID_CONTROLLER_HPP_
