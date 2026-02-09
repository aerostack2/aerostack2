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
 *  \brief      Declares the force estimation behavior.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************************/

#ifndef FORCE_ESTIMATION_BEHAVIOR__FORCE_ESTIMATION_HPP_
#define FORCE_ESTIMATION_BEHAVIOR__FORCE_ESTIMATION_HPP_
#include <vector>
#include <string>

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_msgs/msg/thrust.hpp"


class ForceEstimation
{
public:
  ForceEstimation(
    double alpha,
    size_t n_samples);
  ~ForceEstimation() {}

private:
  double alpha_;
  size_t n_samples_;

public:
  /**
   * @brief Compute thrust from IMU measurements
   * @param current_mass Current mass
   * @param a_z_mean Mean of the last n samples of the z-acceleration
   * @param u_thrust Current thrust command
   * @return Thrust error estimation in Newtons
   */
  double computeThrustError(
    const double & current_mass, const double a_z_mean,
    const double & u_thrust);

  /**
   * @brief Compute mean from a vector of doubles
   * @param vec Vector of doubles
   * @return Mean value
   */
  double computedMeanFromVector(std::vector<double> & vec);

/**
 * @brief Compute mean from the last n samples of a vector of doubles
 * @param vec Vector of doubles
 * @return Mean value
*/
  double computedMeanFromNSamples(const std::vector<double> & vec);

  /**
   * @brief Low-pass filter
   * @param error Input thrust value
   * @param last_filtered_error Last filtered error value
   * @return Filtered thrust value
   */
  double lowPassFiltered(double & thrust, double & last_filtered_error);
};
#endif  // FORCE_ESTIMATION_BEHAVIOR__FORCE_ESTIMATION_HPP_
