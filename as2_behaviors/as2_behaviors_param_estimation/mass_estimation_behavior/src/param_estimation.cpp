/*!*******************************************************************************************
 *  \file       param_estimation.cpp
 *  \brief      Param estimation
 *  \authors    Carmen De Rojas Pita-Romero
 *
 *  \copyright  Copyright (c) 2025 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

 #include "mass_estimation_behavior/param_estimation.hpp"

ParamEstimation::ParamEstimation(
  double initial_mass, double threshold, double alpha,
  size_t n_samples)
{
  last_estimated_mass_ = initial_mass;
  last_filtered_mass_ = initial_mass;
  threshold_ = threshold;
  alpha_ = alpha;
  n_samples_ = n_samples;
}


void ParamEstimation::computeMass(float & thrust, std::vector<double> & a_z)
{
  double mass;
  double a_z_mean = std::abs(computedMeanFromVector(a_z));
  if (a_z_mean > 1e-6 && thrust > 0.0) {
    mass = thrust / a_z_mean;
    if (computeMassError(mass, last_estimated_mass_) ) {
      estimated_mass_ = mass;
      last_estimated_mass_ = estimated_mass_;
    } else {
      estimated_mass_ = last_estimated_mass_;
    }
  } else {
    estimated_mass_ = last_estimated_mass_;
  }
  estimated_mass_vector_.push_back(estimated_mass_);
}

bool ParamEstimation::computeMassError(double & compute_mass, double & last_estimated_mass)
{
  if (threshold_ < 0.0) {
    std::cerr << "[WARN] Invalid threshold value!: " << threshold_ <<
      " No update will be performed." << std::endl;
    return false;
  } else {
    return std::abs(compute_mass - last_estimated_mass) > threshold_;
  }
}


double ParamEstimation::computedMeanFromVector(std::vector<double> & vec)
{
  double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
  return sum / vec.size();
}

double ParamEstimation::computedMeanFromNSamples(const std::vector<double> & vec)
{
  if (vec.empty() || n_samples_ == 0) {return 0.0;}

  size_t count = std::min(n_samples_, vec.size());
  auto start_it = vec.end() - count;

  double sum = std::accumulate(start_it, vec.end(), 0.0);
  return sum / static_cast<double>(count);
}

double ParamEstimation::lowPassFiltered(double & mass)
{
  double filtered_mass = alpha_ * mass + (1 - alpha_) * last_filtered_mass_;
  last_filtered_mass_ = filtered_mass;
  return filtered_mass;
}

// SETTERS
void ParamEstimation::set_threshold(double threshold)
{
  threshold_ = threshold;
}
void ParamEstimation::set_alpha(double alpha)
{
  alpha_ = alpha;
}
void ParamEstimation::set_n_samples(size_t n_samples)
{
  n_samples_ = n_samples;
}

// GETTERS
double ParamEstimation::getEstimatedMass()
{
  double estimated_mass = computedMeanFromNSamples(estimated_mass_vector_);
  estimated_mass_vector_.clear();
  double filtered_mass = lowPassFiltered(estimated_mass);
  return filtered_mass;
}
double ParamEstimation::getThreshold()
{
  return threshold_;
}
double ParamEstimation::getAlpha()
{
  return alpha_;
}
size_t ParamEstimation::getNSamples()
{
  return n_samples_;
}
