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

#include "ekf_fuse/ekf_fuse_ransac_pose.hpp"

#include <random>
#include <stdexcept>
#include <iostream>
#include <unordered_set>
#include <vector>

#include "as2_core/utils/frame_utils.hpp"

std::vector<int> EKFFuseRansacPose::getInliers(
  const std::vector<ekf::PoseMeasurement> & accumulated_poses) const
{
  if (accumulated_poses.empty()) {
    std::cout << "No measurements provided for RANSAC.";
  }

  std::default_random_engine rng(std::random_device{}());
  std::uniform_int_distribution<int> dist(0, accumulated_poses.size() - 1);

  int best_inliers_count = -1;
  std::vector<int> best_inliers;
  int angle_rejections = -1;
  int distance_rejections = -1;
  int both_rejections = -1;

  best_inliers.reserve(accumulated_poses.size());

  for (int it = 0; it < iterations_; ++it) {
    std::vector<int> inliers;
    inliers.reserve(accumulated_poses.size());

    // Pick a random pose as a ground truth hypothesis
    int idx = dist(rng);
    const ekf::PoseMeasurement hypothesis = accumulated_poses[idx];
    std::vector<double> hypothesis_vec{hypothesis.data[ekf::PoseMeasurement::X],
      hypothesis.data[ekf::PoseMeasurement::Y], hypothesis.data[ekf::PoseMeasurement::Z]};

    int angle_rejection_count = 0;
    int distance_rejection_count = 0;
    int both_rejection_count = 0;

    for (int i = 0; i < static_cast<int>(accumulated_poses.size()); i++) {
      bool pose_rejected = false;
      bool angle_rejected = false;

      ekf::PoseMeasurement pose_diff;
      pose_diff.data[ekf::PoseMeasurement::X] = accumulated_poses[i].data[ekf::PoseMeasurement::X] -
        hypothesis.data[ekf::PoseMeasurement::X];
      pose_diff.data[ekf::PoseMeasurement::Y] = accumulated_poses[i].data[ekf::PoseMeasurement::Y] -
        hypothesis.data[ekf::PoseMeasurement::Y];
      pose_diff.data[ekf::PoseMeasurement::Z] = accumulated_poses[i].data[ekf::PoseMeasurement::Z] -
        hypothesis.data[ekf::PoseMeasurement::Z];
      std::vector<double> diff_vec{pose_diff.data[ekf::PoseMeasurement::X],
        pose_diff.data[ekf::PoseMeasurement::Y], pose_diff.data[ekf::PoseMeasurement::Z]};

      double pd =
        std::sqrt(std::inner_product(diff_vec.begin(), diff_vec.end(), diff_vec.begin(), 0.0));
      double od =
        std::abs(
        accumulated_poses[i].data[ekf::PoseMeasurement::YAW] -
        hypothesis.data[ekf::PoseMeasurement::YAW]);

      // if (pd < position_thresh_ && od < orientation_thresh_ * M_PI / 180.0) {
      //     inliers.push_back(i);
      // }

      if (pd > position_thresh_) {
        std::cout << "  [RANSAC] DISTANCE REJECTION -> distance to hypothesis was: " << pd <<
          std::endl;
        distance_rejection_count++;
        pose_rejected = true;
      }

      if (std::abs(od) > orientation_thresh_ * M_PI / 180.0) {
        std::cout << "  [RANSAC] ANGLE REJECTION -> detection: " <<
          (accumulated_poses[i].data[ekf::PoseMeasurement::YAW] * 180.0 / M_PI) <<
          " vs hypothesis: " << (hypothesis.data[ekf::PoseMeasurement::YAW] * 180.0 / M_PI) <<
          std::endl;
        angle_rejection_count++;
        angle_rejected = true;
      }

      if (!(pose_rejected || angle_rejected)) {
        inliers.push_back(i);
      }

      if (pose_rejected && angle_rejected) {
        both_rejection_count++;
      }
    }

    if (static_cast<int>(inliers.size()) > best_inliers_count) {
      best_inliers_count = static_cast<int>(inliers.size());
      best_inliers = inliers;
      distance_rejections = distance_rejection_count;
      angle_rejections = angle_rejection_count;
      both_rejections = both_rejection_count;
    }
  }

  std::cout << "  [RANSAC] Detections rejected due to ANGLE: " << angle_rejections << std::endl;
  std::cout << "  [RANSAC] Detections rejected due to DISTANCE: " << distance_rejections <<
    std::endl;
  std::cout << "  [RANSAC] Detections rejected due to BOTH: " << both_rejections << std::endl;

  return best_inliers;
}

void EKFFuseRansacPose::removeOutliers(
  std::vector<ekf::PoseMeasurement> & accumulated_poses,
  std::vector<ekf::PoseMeasurementCovariance> & accumulated_covs,
  std::vector<std_msgs::msg::Header> & accumulated_headers,
  std::vector<int> inliers)
{
  if (inliers.empty()) {
    return;
  }

  // Convert best_inliers into a fast lookup table
  std::unordered_set<int> keep(inliers.begin(), inliers.end());

  std::vector<ekf::PoseMeasurement> inlier_poses;
  inlier_poses.reserve(keep.size());
  std::vector<ekf::PoseMeasurementCovariance> inlier_covs;
  inlier_covs.reserve(keep.size());
  std::vector<std_msgs::msg::Header> inlier_headers;
  inlier_headers.reserve(keep.size());

  for (int i = 0; i < accumulated_poses.size(); ++i) {
    if (keep.count(i)) {
      inlier_poses.push_back(accumulated_poses[i]);
      inlier_covs.push_back(accumulated_covs[i]);
      inlier_headers.push_back(accumulated_headers[i]);
    }
  }

  accumulated_poses.swap(inlier_poses);
  accumulated_covs.swap(inlier_covs);
  accumulated_headers.swap(inlier_headers);
}
