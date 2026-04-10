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
 * @file polynomial_thrust_map.hpp
 *
 * PolynomialThrustMap class declaration
 *
 * @author Miguel Fernández Cortizas
 *         Francisco José Anguita Chamorro
 *
 */

#ifndef AS2_CORE__POLYNOMIAL_THRUST_MAP_HPP_
#define AS2_CORE__POLYNOMIAL_THRUST_MAP_HPP_

#include <algorithm>
#include <cstdint>
#include <ostream>
#include <vector>
#include <string>
#include "as2_core/aerial_platform.hpp"

namespace as2
{

/**
   * @brief Polynomial thrust map class.
   *
   *  Computes throttle command from thrust and voltage by evaluating:
   *  throttle = a + b * T + c * V + d * T^2 + e * T * V + f * V^2
   */
class PolynomialThrustMap
{
public:
  explicit PolynomialThrustMap(unsigned int n_motors)
  : max_throttle_(2000.0), min_throttle_(1000.0), a(0.0), b(0.0), c(0.0), d(0.0), e(0.0), f(0.0),
    use_correction_factor_(false), gamma2(0.0),
    gamma1(0.0), gamma0(0.0), n_motors(n_motors), platform_node_ptr_(nullptr)
  {}
  explicit PolynomialThrustMap(
    unsigned int n_motors, double a, double b, double c, double d, double e,
    double f, double gamma2, double gamma1, double gamma0, bool use_correction_factor,
    as2::AerialPlatform * platform_node_ptr)
  : max_throttle_(2000.0), min_throttle_(1000.0), a(a), b(b), c(c), d(d), e(e), f(f),
    use_correction_factor_(use_correction_factor),
    gamma2(gamma2), gamma1(gamma1), gamma0(gamma0), n_motors(n_motors),
    platform_node_ptr_(platform_node_ptr)
  {}

  friend std::ostream & operator<<(std::ostream & os, const PolynomialThrustMap & tm)
  {
    os << "ThrustMap: " << tm.a << " " << tm.b << " " << tm.c << " " << tm.d << " " << tm.e <<
      " " << tm.f;
    return os;
  }

  std::string to_string() const;

  template<typename T>
  T getParameter(std::string param_name) const;

  void set_parameters(
    double max_throttle, double min_throttle, double a, double b, double c, double d, double e,
    double f,
    bool use_correction_factor, double gamma2, double gamma1, double gamma0);

  void readParameters();

  void initialize(as2::AerialPlatform * platform_node_ptr);

  double mapThrust(double thrust, double voltage);

  uint16_t getThrottle_useconds(double thrust, double voltage);

  double getThrottle_normalized(double thrust, double voltage);

private:
  // Min and max throttle values for normalized output
  double max_throttle_ = 2000.0;
  double min_throttle_ = 1000.0;
  // Coefficients of the polynomial:
  // throttle = a + b * T + c * V + d * T^2 + e * T * V + f * V^2
  double a, b, c, d, e, f;
  bool use_correction_factor_;
  // Coefficients of the correction factor polynomial:
  // gamma = gamma2 * V^2 + gamma1 * V + gamma0
  double gamma2, gamma1, gamma0;
  uint n_motors;

protected:
  as2::AerialPlatform * platform_node_ptr_;
};  // class PolynomialThrustMap
}  // namespace as2


#endif  // AS2_CORE__POLYNOMIAL_THRUST_MAP_HPP_
