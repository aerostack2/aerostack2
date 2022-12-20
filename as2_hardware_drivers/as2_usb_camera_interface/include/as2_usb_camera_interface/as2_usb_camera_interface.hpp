/*!*******************************************************************************************
 *  \file       usb_camera_interface.hpp
 *  \brief      usb camera interface header file.
 *  \authors    David Perez Saura
 *              Miguel Fernandez Cortizas
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
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

#ifndef USB_CAMERA_INTERFACE_HPP_
#define USB_CAMERA_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class UsbCameraInterface : public as2::Node {
public:
  /**
   * @brief Construct a new UsbCameraInterface object
   */
  UsbCameraInterface();

  /**
   * @brief Destroy the UsbCameraInterface object
   */
  ~UsbCameraInterface(){};

private:
  std::shared_ptr<as2::sensors::Camera> camera_;

  std::string camera_name_;
  std::string camera_frame_;
  std::string device_port_;
  int image_width_;
  int image_height_;
  double framerate_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::string distortion_model_;
  std::string camera_model_;
  std::string encoding_;

  cv::VideoCapture cap_;

  void setCameraParameters(const cv::Mat &_camera_matrix, const cv::Mat &_dist_coeffs);
  void loadParameters();
  void captureImage();
  void setupCamera();
  void setCameraTransform();
  void setCameraModelTransform(const std::string &_camera_flu, const std::string &_camera_rdf);
};

#endif  // USB_CAMERA_INTERFACE_HPP_
