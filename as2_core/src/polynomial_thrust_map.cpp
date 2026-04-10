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
 */

#include "as2_core/polynomial_thrust_map.hpp"

namespace as2
{

void PolynomialThrustMap::initialize(as2::AerialPlatform * platform_node_ptr)
{
  platform_node_ptr_ = platform_node_ptr;
  readParameters();

  RCLCPP_INFO(platform_node_ptr->get_logger(), "Polynomial Thrust Map loaded.");
  RCLCPP_INFO(
    platform_node_ptr->get_logger(), "Poly coefficinets: %s",
    to_string().c_str());
  if (use_correction_factor_) {
    RCLCPP_INFO(
      platform_node_ptr->get_logger(), "Using Correction factor: %s, %s, %s",
      std::to_string(gamma2).c_str(), std::to_string(gamma1).c_str(),
      std::to_string(gamma0).c_str());
  }
}

void PolynomialThrustMap::readParameters()
{
  platform_node_ptr_->declare_parameter<float>("polynomial_thrust_map.a");
  platform_node_ptr_->declare_parameter<float>("polynomial_thrust_map.b");
  platform_node_ptr_->declare_parameter<float>("polynomial_thrust_map.c");
  platform_node_ptr_->declare_parameter<float>("polynomial_thrust_map.d");
  platform_node_ptr_->declare_parameter<float>("polynomial_thrust_map.e");
  platform_node_ptr_->declare_parameter<float>("polynomial_thrust_map.f");
  platform_node_ptr_->declare_parameter<bool>("polynomial_thrust_map.use_correction_factor");
  platform_node_ptr_->declare_parameter<float>("polynomial_thrust_map.gamma2");
  platform_node_ptr_->declare_parameter<float>("polynomial_thrust_map.gamma1");
  platform_node_ptr_->declare_parameter<float>("polynomial_thrust_map.gamma0");

  set_parameters(
    platform_node_ptr_->get_parameter("polynomial_thrust_map.a").as_double(),
    platform_node_ptr_->get_parameter("polynomial_thrust_map.b").as_double(),
    platform_node_ptr_->get_parameter("polynomial_thrust_map.c").as_double(),
    platform_node_ptr_->get_parameter("polynomial_thrust_map.d").as_double(),
    platform_node_ptr_->get_parameter("polynomial_thrust_map.e").as_double(),
    platform_node_ptr_->get_parameter("polynomial_thrust_map.f").as_double(),
    platform_node_ptr_->get_parameter("polynomial_thrust_map.use_correction_factor").as_bool(),
    platform_node_ptr_->get_parameter("polynomial_thrust_map.gamma2").as_double(),
    platform_node_ptr_->get_parameter("polynomial_thrust_map.gamma1").as_double(),
    platform_node_ptr_->get_parameter("polynomial_thrust_map.gamma0").as_double());
}
void PolynomialThrustMap::set_parameters(
  double a, double b, double c, double d, double e, double f,
  bool use_correction_factor, double gamma2, double gamma1, double gamma0)
{
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
  throttle = (throttle < 1000) ? 1000 : throttle;
  throttle = (throttle > 2000) ? 2000 : throttle;
  return throttle;
}

double PolynomialThrustMap::getThrottle_normalized(double thrust, double voltage)
{
  double thrust_per_motor = thrust / static_cast<double>(n_motors);
  double throttle = (mapThrust(thrust_per_motor, voltage) - 1000.0) / 1000.0;
  return std::clamp(throttle, 0.0, 1.0);
}
}  // namespace as2
