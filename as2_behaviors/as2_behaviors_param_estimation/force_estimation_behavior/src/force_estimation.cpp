// Copyright 2026 Universidad Politécnica de Madrid
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
 *  \file       force_estimation_behavior.hpp
 *  \brief      Definition the force estimation.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************************/

# include "force_estimation_behavior/force_estimation.hpp"


ForceEstimation::ForceEstimation(
  double alpha,
  size_t n_samples)
{
  alpha_ = alpha;
  n_samples_ = n_samples;
}

double ForceEstimation::computeThrustError(
  const double & current_mass, const double a_z_mean,
  const double & u_thrust)
{
  double measured_thrust;
  double thrust_error;
  if (a_z_mean > 1e-6) {
    measured_thrust = current_mass * a_z_mean;
    thrust_error = std::abs(u_thrust - measured_thrust);
    return thrust_error;
  }
  return 0.0;
}

double ForceEstimation::computedMeanFromVector(std::vector<double> & vec)
{
  double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
  return sum / vec.size();
}

double ForceEstimation::computedMeanFromNSamples(const std::vector<double> & vec)
{
  if (vec.empty() || n_samples_ == 0) {return 0.0;}

  size_t count = std::min(n_samples_, vec.size());
  auto start_it = vec.end() - count;

  double sum = std::accumulate(start_it, vec.end(), 0.0);
  return sum / static_cast<double>(count);
}

double ForceEstimation::lowPassFiltered(double & force_error, double & last_filtered_force_error)
{
  double filtered_force = alpha_ * force_error + (1 - alpha_) * last_filtered_force_error;
  return filtered_force;
}
