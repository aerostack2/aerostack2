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

/*!******************************************************************************
 *  \file       img_preprocessing.hpp
 *  \brief      image preprocessing header file.
 *  \authors    Alba López del Águila
 ********************************************************************************/

#ifndef AS2_BEHAVIORS_OBJECT_PERCEPTION__IMAGE_PREPROCESSING_HPP_
#define AS2_BEHAVIORS_OBJECT_PERCEPTION__IMAGE_PREPROCESSING_HPP_

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudawarping.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace as2_behaviors_object_perception
{

class ImagePreprocessor
{
public:
  explicit ImagePreprocessor(const rclcpp::Logger & logger);
  ~ImagePreprocessor() = default;

  void setRectificationEnabled(bool enable);
  bool rectificationEnabled() const;

  bool updateCameraInfo(const sensor_msgs::msg::CameraInfo & camera_info);

  bool preprocessCompressedImage(
    const sensor_msgs::msg::CompressedImage & image_msg,
    cv::Mat & output_image);

  bool hasCameraInfo() const;
  bool rectificationReady() const;

  const sensor_msgs::msg::CameraInfo & getCameraInfo() const;

private:
  bool decodeCompressedImage(
    const sensor_msgs::msg::CompressedImage & image_msg,
    cv::Mat & decoded_image);

  bool rectifyImage(
    const cv::Mat & input_image,
    cv::Mat & output_image);

  bool needsRectification() const;

  void initRectificationMaps(const cv::Size & size);

private:
  rclcpp::Logger logger_;

  cv::Mat buffer_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::string distortion_model_;

  bool enable_rectification_{false};
  bool significant_distortion_{false};
  bool camera_info_received_{false};

  cv::Mat map1_;
  cv::Mat map2_;
  cv::cuda::GpuMat d_map1_;
  cv::cuda::GpuMat d_map2_;
  cv::Size rectified_size_;
  bool rect_maps_initialized_{false};

  sensor_msgs::msg::CameraInfo camera_info_;
};

}  // namespace as2_behaviors_object_perception

#endif
