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
 * @file thrust_map.hpp
 *
 * ThrustMap class declaration
 *
 * @author Miguel Fernández Cortizas
 *
 */

#ifndef AS2_CORE__THRUST_MAP_HPP_
#define AS2_CORE__THRUST_MAP_HPP_

#include <algorithm>
#include <cstdint>
#include <ostream>
#include <string>

namespace as2
{

class ThrustMap
{
public:
  explicit ThrustMap(unsigned int n_motors)
  : a(0.0), b(0.0), c(0.0), d(0.0), e(0.0), f(0.0), n_motors(n_motors), gamma2(0.0), gamma1(0.0),
    gamma0(0.0), use_correction_factor(false)
  {}
  explicit ThrustMap(
    unsigned int n_motors, double a, double b, double c, double d, double e,
    double f, double gamma2, double gamma1, double gamma0, bool use_correction_factor)
  : a(a), b(b), c(c), d(d), e(e), f(f), n_motors(n_motors), gamma2(gamma2), gamma1(gamma1), gamma0(
      gamma0), use_correction_factor(use_correction_factor)
  {}

  void set_parameters(
    double a, double b, double c, double d, double e, double f,
    bool use_correction_factor, double gamma2, double gamma1, double gamma0);

  friend std::ostream & operator<<(std::ostream & os, const ThrustMap & tm)
  {
    os << "ThrustMap: " << tm.a << " " << tm.b << " " << tm.c << " " << tm.d << " " << tm.e <<
      " " << tm.f;
    return os;
  }

  std::string to_string() const
  {
    if (!use_correction_factor) {
      return "ThrustMap: " + std::to_string(a) + " " + std::to_string(b) + " " + std::to_string(c) +
             " " + std::to_string(d) + " " + std::to_string(e) + " " + std::to_string(f);
    } else {
      return "ThrustMap: " + std::to_string(a) + " " + std::to_string(b) + " " + std::to_string(c) +
             " " + std::to_string(d) + " " + std::to_string(e) + " " + std::to_string(f) +
             "\nCorrection factor: " + std::to_string(gamma2) + " " + std::to_string(gamma1) + " " +
             std::to_string(gamma0);
    }
  }

  double mapThrust(double thrust, double voltage);

  uint16_t getThrottle_useconds(double thrust, double voltage);

  double getThrottle_normalized(double thrust, double voltage);

private:
  double a, b, c, d, e, f;
  uint n_motors;
  double gamma2, gamma1, gamma0;
  bool use_correction_factor;
};  // class ThrustMap
}  // namespace as2


#endif  // AS2_CORE__THRUST_MAP_HPP_
