// Copyright 2025 Universidad Politécnica de Madrid
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

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <ekf/ekf_datatype.hpp>

#include "std_msgs/msg/header.hpp"

class EKFFuseRansacPose
{
public:
  EKFFuseRansacPose() = default;
  EKFFuseRansacPose(int iterations, double pos_thresh, double yaw_thresh)
  : iterations_(iterations),
    position_thresh_(pos_thresh),
    orientation_thresh_(yaw_thresh)
  {}

  void configure(int iterations, double pos_thresh, double ori_thresh)
  {
    iterations_ = iterations;
    position_thresh_ = pos_thresh;
    orientation_thresh_ = ori_thresh;
  }

  void setIterations(int it) {iterations_ = it;}
  void setPositionThreshold(double t) {position_thresh_ = t;}
  void setOrientationThreshold(double t_deg) {orientation_thresh_ = t_deg;}

  int iterations() const {return iterations_;}
  double positionThreshold() const {return position_thresh_;}
  double orientationThreshold() const {return orientation_thresh_;}

  std::vector<int> getInliers(const std::vector<ekf::PoseMeasurement> & accumulated_poses) const;
  void removeOutliers(
    std::vector<ekf::PoseMeasurement> & accumulated_poses,
    std::vector<ekf::PoseMeasurementCovariance> & accumulated_covs,
    std::vector<std_msgs::msg::Header> & accumulated_headers,
    std::vector<int> inliers);

private:
  int iterations_;
  double position_thresh_;
  double orientation_thresh_;
};
