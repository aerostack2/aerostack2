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
 *  \file       image_preprocessing.cpp
 *  \brief      image preprocessing source file.
 *  \authors    Alba López del Águila
 ********************************************************************************/

#include "as2_behaviors_object_perception/common/img_preprocessing.hpp"

#include <algorithm>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace as2_behaviors_object_perception
{

ImagePreprocessor::ImagePreprocessor(const rclcpp::Logger & logger)
: logger_(logger)
{
}

void ImagePreprocessor::setRectificationEnabled(bool enable)
{
  enable_rectification_ = enable;
}

bool ImagePreprocessor::rectificationEnabled() const
{
  return enable_rectification_;
}

bool ImagePreprocessor::hasCameraInfo() const
{
  return camera_info_received_;
}

bool ImagePreprocessor::rectificationReady() const
{
  return rect_maps_initialized_;
}

const sensor_msgs::msg::CameraInfo & ImagePreprocessor::getCameraInfo() const
{
  return camera_info_;
}

bool ImagePreprocessor::updateCameraInfo(const sensor_msgs::msg::CameraInfo & camera_info)
{
  if (camera_info.k.size() != 9) {
    RCLCPP_ERROR(logger_, "Invalid camera info: K must have 9 elements");
    return false;
  }

  camera_info_ = camera_info;
  distortion_model_ = camera_info.distortion_model;

  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (size_t i = 0; i < 9; ++i) {
    camera_matrix_.at<double>(static_cast<int>(i / 3), static_cast<int>(i % 3)) = camera_info.k[i];
  }

  if (!camera_info.d.empty()) {
    dist_coeffs_ = cv::Mat(
      static_cast<int>(camera_info.d.size()), 1, CV_64F);
    for (size_t i = 0; i < camera_info.d.size(); ++i) {
      dist_coeffs_.at<double>(static_cast<int>(i), 0) = camera_info.d[i];
    }
  } else {
    dist_coeffs_ = cv::Mat();
  }

  significant_distortion_ = false;
  if (!dist_coeffs_.empty()) {
    for (int i = 0; i < dist_coeffs_.rows; ++i) {
      if (std::abs(dist_coeffs_.at<double>(i, 0)) > 1e-9) {
        significant_distortion_ = true;
        break;
      }
    }
  }

  camera_info_received_ = true;
  rect_maps_initialized_ = false;
  map1_.release();
  map2_.release();
  d_map1_.release();
  d_map2_.release();
  rectified_size_ = cv::Size();

  RCLCPP_INFO(
    logger_,
    "Camera info updated. Distortion model: '%s', significant distortion: %s",
    distortion_model_.c_str(),
    significant_distortion_ ? "true" : "false");

  return true;
}

bool ImagePreprocessor::preprocessCompressedImage(
  const sensor_msgs::msg::CompressedImage & image_msg,
  cv::Mat & output_image)
{
  cv::Mat decoded_image;
  if (!decodeCompressedImage(image_msg, decoded_image)) {
    return false;
  }

  if (needsRectification()) {
    if (!rect_maps_initialized_ || rectified_size_ != decoded_image.size()) {
      initRectificationMaps(decoded_image.size());
    }

    if (rect_maps_initialized_) {
      return rectifyImage(decoded_image, output_image);
    }
  }

  output_image = decoded_image;
  return true;
}

bool ImagePreprocessor::decodeCompressedImage(
  const sensor_msgs::msg::CompressedImage & image_msg,
  cv::Mat & decoded_image)
{
  if (image_msg.data.empty()) {
    RCLCPP_ERROR(logger_, "Compressed image data is empty");
    return false;
  }

  buffer_ = cv::Mat(
    1,
    static_cast<int>(image_msg.data.size()),
    CV_8UC1,
    const_cast<unsigned char *>(image_msg.data.data()));

  decoded_image = cv::imdecode(buffer_, cv::IMREAD_COLOR);

  if (decoded_image.empty()) {
    RCLCPP_ERROR(logger_, "Failed to decode compressed image");
    return false;
  }

  return true;
}

bool ImagePreprocessor::rectifyImage(
  const cv::Mat & input_image,
  cv::Mat & output_image)
{
  if (!rect_maps_initialized_) {
    RCLCPP_WARN(logger_, "Rectification requested but maps are not initialized");
    output_image = input_image;
    return false;
  }

  try {
    cv::cuda::GpuMat d_src;
    cv::cuda::GpuMat d_dst;
    d_src.upload(input_image);

    cv::cuda::remap(
      d_src,
      d_dst,
      d_map1_,
      d_map2_,
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

    d_dst.download(output_image);
    return true;
  } catch (const cv::Exception & e) {
    RCLCPP_WARN(
      logger_,
      "CUDA rectification failed: %s. Falling back to CPU rectification.",
      e.what());
  }

  try {
    cv::remap(
      input_image,
      output_image,
      map1_,
      map2_,
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);
    return true;
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR(
      logger_,
      "CPU rectification failed: %s. Returning original image.",
      e.what());
    output_image = input_image;
    return false;
  }
}

bool ImagePreprocessor::needsRectification() const
{
  return enable_rectification_ &&
         camera_info_received_ &&
         significant_distortion_ &&
         !camera_matrix_.empty() &&
         !dist_coeffs_.empty();
}

void ImagePreprocessor::initRectificationMaps(const cv::Size & size)
{
  if (camera_matrix_.empty() || dist_coeffs_.empty()) {
    RCLCPP_WARN(
      logger_,
      "Cannot initialize rectification maps: camera matrix or distortion coefficients are empty");
    rect_maps_initialized_ = false;
    return;
  }

  rectified_size_ = size;

  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat P = camera_matrix_.clone();

  try {
    if (distortion_model_ == "fisheye" || distortion_model_ == "equidistant") {
      cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        camera_matrix_,
        dist_coeffs_,
        rectified_size_,
        R,
        P,
        0.0);

      cv::fisheye::initUndistortRectifyMap(
        camera_matrix_,
        dist_coeffs_,
        R,
        P,
        rectified_size_,
        CV_32FC1,
        map1_,
        map2_);

      if (P.rows >= 3 && P.cols >= 3) {
        camera_matrix_ = P(cv::Rect(0, 0, 3, 3)).clone();

        if (camera_info_.k.size() == 9) {
          camera_info_.k[0] = camera_matrix_.at<double>(0, 0);
          camera_info_.k[1] = camera_matrix_.at<double>(0, 1);
          camera_info_.k[2] = camera_matrix_.at<double>(0, 2);
          camera_info_.k[3] = camera_matrix_.at<double>(1, 0);
          camera_info_.k[4] = camera_matrix_.at<double>(1, 1);
          camera_info_.k[5] = camera_matrix_.at<double>(1, 2);
          camera_info_.k[6] = camera_matrix_.at<double>(2, 0);
          camera_info_.k[7] = camera_matrix_.at<double>(2, 1);
          camera_info_.k[8] = camera_matrix_.at<double>(2, 2);
        }

        if (camera_info_.p.size() == 12) {
          camera_info_.p[0] = P.at<double>(0, 0);
          camera_info_.p[1] = P.at<double>(0, 1);
          camera_info_.p[2] = P.at<double>(0, 2);
          camera_info_.p[3] = P.at<double>(0, 3);
          camera_info_.p[4] = P.at<double>(1, 0);
          camera_info_.p[5] = P.at<double>(1, 1);
          camera_info_.p[6] = P.at<double>(1, 2);
          camera_info_.p[7] = P.at<double>(1, 3);
          camera_info_.p[8] = P.at<double>(2, 0);
          camera_info_.p[9] = P.at<double>(2, 1);
          camera_info_.p[10] = P.at<double>(2, 2);
          camera_info_.p[11] = P.at<double>(2, 3);
        }

        camera_info_.d.assign(camera_info_.d.size(), 0.0);
      }

      RCLCPP_INFO(logger_, "Initialized fisheye rectification maps");
    } else if (
      distortion_model_ == "plumb_bob" ||
      distortion_model_ == "radtan" ||
      distortion_model_.empty())
    {
      cv::initUndistortRectifyMap(
        camera_matrix_,
        dist_coeffs_,
        R,
        P,
        rectified_size_,
        CV_32FC1,
        map1_,
        map2_);

      RCLCPP_INFO(logger_, "Initialized plumb_bob rectification maps");
    } else {
      RCLCPP_WARN(
        logger_,
        "Unknown distortion model '%s'. Rectification disabled for this stream.",
        distortion_model_.c_str());
      rect_maps_initialized_ = false;
      return;
    }

    try {
      d_map1_.upload(map1_);
      d_map2_.upload(map2_);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(
        logger_,
        "Failed to upload rectification maps to CUDA: %s. CPU fallback will be used.",
        e.what());
      d_map1_.release();
      d_map2_.release();
    }

    rect_maps_initialized_ = true;
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR(
      logger_,
      "Failed to initialize rectification maps: %s",
      e.what());
    rect_maps_initialized_ = false;
  }
}

}  // namespace as2_behaviors_object_perception
