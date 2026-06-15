// Copyright 2023 Universidad Politécnica de Madrid
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

/**
 * @file thrust_map.cpp
 *
 * @author Miguel Fernández Cortizas
 *         Francisco José Anguita Chamorro
 */

#include "as2_core/polynomial_thrust_map.hpp"

namespace as2
{

std::string PolynomialThrustMap::to_string() const
{
  std::string tm_string = "Thrust Map with output range [" + std::to_string(min_throttle_) + ", " +
    std::to_string(max_throttle_) + "]. \nCoefficients: " + std::to_string(a) + " " +
    std::to_string(b) + " " + std::to_string(c) + " " + std::to_string(d) + " " +
    std::to_string(e) + " " + std::to_string(f);
  if (use_correction_factor_) {
    tm_string += "\nCorrection factor: " + std::to_string(gamma2) + " " + std::to_string(gamma1) +
      " " + std::to_string(gamma0);
  }
  return tm_string;
}

template<typename T>
T PolynomialThrustMap::getParameter(std::string param_name) const
{
  if (!platform_node_ptr_->has_parameter(param_name)) {
    platform_node_ptr_->declare_parameter<T>(param_name);
  }
  return platform_node_ptr_->get_parameter(param_name).get_value<T>();
}

void PolynomialThrustMap::set_parameters(
  double max_throttle, double min_throttle, double a, double b, double c, double d, double e,
  double f,
  bool use_correction_factor, double gamma2, double gamma1, double gamma0)
{
  this->max_throttle_ = max_throttle;
  this->min_throttle_ = min_throttle;
  this->a = a;
  this->b = b;
  this->c = c;
  this->d = d;
  this->e = e;
  this->f = f;
  this->use_correction_factor_ = use_correction_factor;
  this->gamma2 = gamma2;
  this->gamma1 = gamma1;
  this->gamma0 = gamma0;
}

void PolynomialThrustMap::readParameters()
{
  set_parameters(
    getParameter<double>("polynomial_thrust_map.max_throttle"),
    getParameter<double>("polynomial_thrust_map.min_throttle"),
    getParameter<double>("polynomial_thrust_map.a"),
    getParameter<double>("polynomial_thrust_map.b"),
    getParameter<double>("polynomial_thrust_map.c"),
    getParameter<double>("polynomial_thrust_map.d"),
    getParameter<double>("polynomial_thrust_map.e"),
    getParameter<double>("polynomial_thrust_map.f"),
    getParameter<bool>("polynomial_thrust_map.use_correction_factor"),
    getParameter<double>("polynomial_thrust_map.gamma2"),
    getParameter<double>("polynomial_thrust_map.gamma1"),
    getParameter<double>("polynomial_thrust_map.gamma0"));
}

void PolynomialThrustMap::initialize(as2::AerialPlatform * platform_node_ptr)
{
  platform_node_ptr_ = platform_node_ptr;
  readParameters();

  RCLCPP_INFO(platform_node_ptr->get_logger(), "Polynomial Thrust Map loaded.");
  RCLCPP_INFO(
    platform_node_ptr->get_logger(), "%s",
    to_string().c_str());
  if (use_correction_factor_) {
    RCLCPP_INFO(
      platform_node_ptr->get_logger(), "Using Correction factor: %s, %s, %s",
      std::to_string(gamma2).c_str(), std::to_string(gamma1).c_str(),
      std::to_string(gamma0).c_str());
  }
}

double PolynomialThrustMap::mapThrust(double thrust, double voltage)
{
  double x = thrust;
  double y = voltage;

  return a + b * x + c * y + d * x * x + e * x * y + f * y * y;
}

uint16_t PolynomialThrustMap::getThrottle_useconds(double thrust, double voltage)
{
  double thrust_per_motor = thrust / static_cast<double>(n_motors);

  if (use_correction_factor_) {
    double gamma = gamma2 * voltage * voltage + gamma1 * voltage + gamma0;
    thrust_per_motor = gamma * thrust_per_motor;
  }

  uint16_t throttle = static_cast<uint16_t>(mapThrust(thrust_per_motor, voltage));
  throttle = (throttle < min_throttle_) ? min_throttle_ : throttle;
  throttle = (throttle > max_throttle_) ? max_throttle_ : throttle;
  return throttle;
}

double PolynomialThrustMap::getThrottle_normalized(double thrust, double voltage)
{
  double thrust_per_motor = thrust / static_cast<double>(n_motors);
  double throttle =
    (mapThrust(thrust_per_motor, voltage) - min_throttle_) / (max_throttle_ - min_throttle_);
  return std::clamp(throttle, 0.0, 1.0);
}
}  // namespace as2
