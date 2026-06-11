// Copyright 2024 Universidad Politécnica de Madrid
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
* @file as2_usb_camera_interface.cpp
*
* @brief Implementation of the generic camera interface
*
* @authors David Perez Saura, Miguel Fernandez Cortizas
*/

#include "as2_usb_camera_interface.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "as2_core/utils/tf_utils.hpp"

namespace usb_camera_interface
{

UsbCameraInterface::UsbCameraInterface(as2::Node * node_ptr)
: node_ptr_(node_ptr)
{
  // as2::sensors::Camera reads the camera_info and the camera TF from ROS
  // parameters (when the "camera_name" parameter is set) and handles the
  // image / camera_info publishing.
  camera_ = std::make_shared<as2::sensors::Camera>(node_ptr_);

  setupCamera();
  cameraInfoSetup();

  publish_images_ = as2_usb_camera_interface::getParameter<bool>(node_ptr_, "publish_images");

  const int output_queue_size =
    as2_usb_camera_interface::getParameterOr<int>(node_ptr_, "output_queue_size", 1);
  output_queue_.setMaxSize(output_queue_size > 0 ? static_cast<size_t>(output_queue_size) : 1);

  const int64_t milliseconds_from_framerate =
    static_cast<int64_t>(1000.0 / framerate_);
  capture_callback_group_ = node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  image_capture_timer_ = node_ptr_->create_timer(
    std::chrono::milliseconds(milliseconds_from_framerate),
    std::bind(&UsbCameraInterface::captureImage, this),
    capture_callback_group_);
}

void UsbCameraInterface::setupCamera()
{
  bool arducam = false;
  std::string device_port;
  double framerate = 30.0;
  int image_width = 0;
  int image_height = 0;

  // "image_width"/"image_height"/"camera_name" are already declared by the
  // as2::sensors::Camera constructor; the getParameter helper only reads them.
  arducam = as2_usb_camera_interface::getParameter<bool>(node_ptr_, "arducam");
  device_port = as2_usb_camera_interface::getParameter<std::string>(node_ptr_, "device");
  framerate = as2_usb_camera_interface::getParameter<double>(node_ptr_, "framerate");
  image_height = as2_usb_camera_interface::getParameter<int>(node_ptr_, "image_height");
  image_width = as2_usb_camera_interface::getParameter<int>(node_ptr_, "image_width");
  camera_name_ = as2_usb_camera_interface::getParameter<std::string>(node_ptr_, "camera_name");
  framerate_ = framerate;

  RCLCPP_INFO(node_ptr_->get_logger(), "Video device: %s", device_port.c_str());

  if (arducam) {
    // Jetson CSI Arducam backend via GStreamer nvarguscamerasrc.
    RCLCPP_INFO(node_ptr_->get_logger(), "Using arducam (GStreamer) backend");
    std::string image_width_str = std::to_string(image_width);
    std::string image_height_str = std::to_string(image_height);
    int framerate_int = static_cast<int>(std::round(framerate));
    std::string framerate_str = std::to_string(framerate_int);

    // Optional nvarguscamerasrc image controls, built like as2_gates_localization:
    // ranges/compensation/wbmode/saturation are only set when they differ from the
    // sensor default, so leaving them at the default keeps the auto behaviour.
    namespace ns = as2_usb_camera_interface;
    const double exposure_recompensation =
      ns::getParameterOr<double>(node_ptr_, "exposure_recompensation", 0.0);
    const int exposure_range = ns::getParameterOr<int>(node_ptr_, "exposure_range", 0);
    const int gain = ns::getParameterOr<int>(node_ptr_, "gain", 0);
    const bool aelock = ns::getParameterOr<bool>(node_ptr_, "aelock", true);
    const bool awblock = ns::getParameterOr<bool>(node_ptr_, "awblock", false);
    const int wbmode = ns::getParameterOr<int>(node_ptr_, "wbmode", 1);
    const int aeantibanding = ns::getParameterOr<int>(node_ptr_, "aeantibanding", 1);
    const double saturation = ns::getParameterOr<double>(node_ptr_, "saturation", 1.0);

    std::string source = "nvarguscamerasrc sensor-id=" + device_port;
    if (exposure_recompensation != 0.0) {
      source += " exposurecompensation=" + std::to_string(exposure_recompensation);
    }
    if (exposure_range != 0) {
      const std::string e = std::to_string(exposure_range);
      source += " exposuretimerange=\"" + e + " " + e + "\"";
    }
    if (gain != 0) {
      const std::string g = std::to_string(gain);
      source += " gainrange=\"" + g + " " + g + "\"";
    }
    source += " aelock=" + std::string(aelock ? "1" : "0");
    source += " awblock=" + std::string(awblock ? "1" : "0");
    if (wbmode != 1) {
      source += " wbmode=" + std::to_string(wbmode);
    }
    source += " aeantibanding=" + std::to_string(aeantibanding);
    if (saturation != 1.0) {
      source += " saturation=" + std::to_string(saturation);
    }

    auto device_full_name = source +
      " ! video/x-raw(memory:NVMM), width=(int)" + image_width_str + ", height=(int)" +
      image_height_str + ",format=(string)NV12, framerate=(fraction)" + framerate_str +
      "/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw,format=(string)BGR ! appsink drop=1";  // NOLINT
    RCLCPP_INFO(node_ptr_->get_logger(), "Device full name: %s", device_full_name.c_str());
    cap_ = cv::VideoCapture(device_full_name, cv::CAP_GSTREAMER);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Cannot open device");
      return;
    }
  } else {
    // Generic OpenCV backend: any USB / V4L2 camera.
    // "device" may be a path ("/dev/video0") or a numeric index ("0").
    const bool numeric_device = !device_port.empty() &&
      std::all_of(device_port.begin(), device_port.end(), ::isdigit);
    if (numeric_device) {
      cap_.open(std::stoi(device_port));
    } else {
      cap_.open(device_port);
    }
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Cannot open device");
      return;
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
    cap_.set(cv::CAP_PROP_FPS, framerate);
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Camera capture setup complete");
}

void UsbCameraInterface::cameraInfoSetup()
{
  // Mirror the intrinsics read by as2::sensors::Camera so in-process consumers
  // (via getCameraInfoMessage) get the same calibration that is published.
  camera_info_.width = as2_usb_camera_interface::getParameter<int>(node_ptr_, "image_width");
  camera_info_.height = as2_usb_camera_interface::getParameter<int>(node_ptr_, "image_height");
  camera_info_.distortion_model =
    as2_usb_camera_interface::getParameter<std::string>(node_ptr_, "distortion_model");
  convertVectorToArray(
    as2_usb_camera_interface::getParameter<std::vector<double>>(node_ptr_, "camera_matrix.data"),
    camera_info_.k);
  convertVectorToArray(
    as2_usb_camera_interface::getParameter<std::vector<double>>(
      node_ptr_, "projection_matrix.data"), camera_info_.p);
  convertVectorToArray(
    as2_usb_camera_interface::getParameter<std::vector<double>>(
      node_ptr_, "rectification_matrix.data"), camera_info_.r);
  camera_info_.d = as2_usb_camera_interface::getParameter<std::vector<double>>(
    node_ptr_, "distortion_coefficients.data");
  camera_info_.header.frame_id = as2::tf::generateTfName(
    node_ptr_->get_namespace(), camera_name_ + "/camera_link");
}

void UsbCameraInterface::captureImage()
{
  cv::Mat frame;
  if (!cap_.read(frame)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Cannot read image");
    return;
  }

  if (publish_images_) {
    camera_->updateData(frame);
  }

  CameraFrame camera_frame;
  camera_frame.image = frame;
  camera_frame.header.stamp = node_ptr_->now();
  camera_frame.header.frame_id = as2::tf::generateTfName(
    node_ptr_->get_namespace(), camera_name_ + "/camera_link");
  output_queue_.push(camera_frame);
}

}  // namespace usb_camera_interface
