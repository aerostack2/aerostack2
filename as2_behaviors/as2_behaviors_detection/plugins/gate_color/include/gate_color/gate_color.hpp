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

/*!******************************************************************************
 *  \file       detect_behavior_plugin_base.hpp
 *  \brief      detect_behavior_plugin_base header file.
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************/

#ifndef GATE_COLOR__GATE_COLOR_HPP_
#define GATE_COLOR__GATE_COLOR_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/opencv.hpp>


#include "as2_behaviors_detection/as2_behaviors_detection_plugin_base.hpp"

namespace gate_color
{
class Plugin : public as2_behaviors_detection_plugin_base::DetectBase
{
private:
  float gate_width_;
  float gate_height_;
  std::vector<int64_t> gate_color_;
  std::vector<int64_t> gate_color_tolerance_;
  int min_cont_size_;
  float aspect_ratio_th;

  // Preprocessing
  cv::Mat buffer_;
  bool enable_rectification_ = false;
  bool significant_distorsion_ = false;
  cv::Mat map1_, map2_;
  cv::cuda::GpuMat d_map1_, d_map2_;

public:
  Plugin() = default;
  ~Plugin() override = default;

  void ownInit() override;

  bool own_activate(as2_msgs::action::Detect::Goal & goal) override;
  bool own_modify(as2_msgs::action::Detect::Goal & goal) override;
  bool own_deactivate(const std::shared_ptr<std::string> & message) override;
  bool own_pause(const std::shared_ptr<std::string> & message) override;
  bool own_resume(const std::shared_ptr<std::string> & message) override;

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override;
  as2_behavior::ExecutionStatus own_run() override;

  std::array<cv::Point, 4> getCorners(const std::vector<cv::Point> & approx);

  void compressedImagePreprocessing(
    const sensor_msgs::msg::CompressedImage::SharedPtr camera_image_msg,
    cv::Mat & image);

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image_msg) override;

  void localizeGate(const std::array<cv::Point, 4> & corners);
};
}  // namespace gate_color
#endif
