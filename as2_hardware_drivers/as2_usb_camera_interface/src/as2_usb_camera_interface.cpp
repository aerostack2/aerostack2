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
#include <filesystem>
#include <string>
#include <system_error>
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

  publish_images_ = node_ptr_->getParameter<bool>("publish_images");

  const int output_queue_size =
    node_ptr_->getParameter<int>("output_queue_size", 1);
  output_queue_.setMaxSize(output_queue_size > 0 ? static_cast<size_t>(output_queue_size) : 1);

  // With "image_folder" set, every frame is also saved there as <stamp in ns>.bmp. "save_threads"
  // writer threads do the disk writes so they never delay the capture.
  image_folder_ = node_ptr_->getParameter<std::string>("image_folder", "");
  if (!image_folder_.empty()) {
    std::error_code error;
    std::filesystem::create_directories(image_folder_, error);
    if (error) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Cannot create image_folder '%s': %s. Images will not be saved",
        image_folder_.c_str(), error.message().c_str());
      image_folder_.clear();
    } else {
      const int save_threads = node_ptr_->getParameter<int>("save_threads", 1);
      for (int i = 0; i < std::max(save_threads, 1); ++i) {
        image_writer_threads_.emplace_back(&UsbCameraInterface::saveImages, this);
      }
    }
  }

  // The camera runs at "camera_framerate"; frames are read and published at "publish_hz".
  if (publish_hz_ <= 0.0) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "publish_hz must be > 0: set publish_hz (or the legacy framerate). No images will be read");
    return;
  }
  const int64_t milliseconds_from_publish_hz =
    static_cast<int64_t>(1000.0 / publish_hz_);
  capture_callback_group_ = node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  image_capture_timer_ = node_ptr_->create_timer(
    std::chrono::milliseconds(milliseconds_from_publish_hz),
    std::bind(&UsbCameraInterface::captureImage, this),
    capture_callback_group_);
}

UsbCameraInterface::~UsbCameraInterface()
{
  {
    std::lock_guard<std::mutex> lock(images_to_save_mutex_);
    stop_image_writers_ = true;
  }
  // Wake every writer: each one returns only once the queue is empty, so no queued image is lost
  images_to_save_cv_.notify_all();
  for (auto & thread : image_writer_threads_) {
    thread.join();
  }
}

void UsbCameraInterface::setupCamera()
{
  bool arducam = false;
  std::string device_port;
  double camera_framerate = 30.0;
  int image_width = 0;
  int image_height = 0;

  // "image_width"/"image_height"/"camera_name" are already declared by the
  // as2::sensors::Camera constructor; the getParameter helper only reads them.
  arducam = node_ptr_->getParameter<bool>("arducam");
  device_port = node_ptr_->getParameter<std::string>("device");
  // The legacy "framerate" is used for "camera_framerate" / "publish_hz" when they are not set.
  const double framerate = node_ptr_->getParameter<double>("framerate", 0.0);
  camera_framerate = node_ptr_->getParameter<double>("camera_framerate", framerate);
  publish_hz_ = node_ptr_->getParameter<double>("publish_hz", framerate);
  image_height = node_ptr_->getParameter<int>("image_height");
  image_width = node_ptr_->getParameter<int>("image_width");
  camera_name_ = node_ptr_->getParameter<std::string>("camera_name");

  if (camera_framerate <= 0.0) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "camera_framerate must be > 0: set camera_framerate (or the legacy framerate)");
    return;
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Video device: %s", device_port.c_str());

  if (arducam) {
    // Jetson CSI Arducam backend via GStreamer nvarguscamerasrc.
    RCLCPP_INFO(node_ptr_->get_logger(), "Using arducam (GStreamer) backend");
    std::string image_width_str = std::to_string(image_width);
    std::string image_height_str = std::to_string(image_height);
    int framerate_int = static_cast<int>(std::round(camera_framerate));
    std::string framerate_str = std::to_string(framerate_int);

    // Optional nvarguscamerasrc image controls, built like as2_gates_localization:
    // ranges/compensation/wbmode/saturation are only set when they differ from the
    // sensor default, so leaving them at the default keeps the auto behaviour.
    const double exposure_recompensation =
      node_ptr_->getParameter<double>("exposure_recompensation", 0.0);
    const int exposure_range = node_ptr_->getParameter<int>("exposure_range", 0);
    const int gain = node_ptr_->getParameter<int>("gain", 0);
    const bool aelock = node_ptr_->getParameter<bool>("aelock", true);
    const bool awblock = node_ptr_->getParameter<bool>("awblock", false);
    const int wbmode = node_ptr_->getParameter<int>("wbmode", 1);
    const int aeantibanding = node_ptr_->getParameter<int>("aeantibanding", 1);
    const double saturation = node_ptr_->getParameter<double>("saturation", 1.0);

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
      "/1 ! nvvidconv ! video/x-raw, format=(string)BGRx " +
      "! videoconvert ! video/x-raw,format=(string)BGR ! appsink drop=1";
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
    cap_.set(cv::CAP_PROP_FPS, camera_framerate);
    // Keep a single driver buffer so reads at publish_hz < camera_framerate get the newest frame.
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Camera capture setup complete");
}

void UsbCameraInterface::cameraInfoSetup()
{
  // Mirror the intrinsics read by as2::sensors::Camera so in-process consumers
  // (via getCameraInfoMessage) get the same calibration that is published.
  camera_info_.width = node_ptr_->getParameter<int>("image_width");
  camera_info_.height = node_ptr_->getParameter<int>("image_height");
  camera_info_.distortion_model =
    node_ptr_->getParameter<std::string>("distortion_model");
  convertVectorToArray(
    node_ptr_->getParameter<std::vector<double>>("camera_matrix.data"),
    camera_info_.k);
  convertVectorToArray(
    node_ptr_->getParameter<std::vector<double>>("projection_matrix.data"),
    camera_info_.p);
  convertVectorToArray(
    node_ptr_->getParameter<std::vector<double>>("rectification_matrix.data"),
    camera_info_.r);
  camera_info_.d =
    node_ptr_->getParameter<std::vector<double>>("distortion_coefficients.data");
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

  // Stamped right after the read, so the stamp is the capture time and not delayed by publishing.
  CameraFrame camera_frame;
  camera_frame.image = frame;
  camera_frame.header.stamp = node_ptr_->now();
  camera_frame.header.frame_id = as2::tf::generateTfName(
    node_ptr_->get_namespace(), camera_name_ + "/camera_link");

  if (!image_folder_.empty()) {
    size_t images_waiting = 0;
    {
      std::lock_guard<std::mutex> lock(images_to_save_mutex_);
      images_to_save_.push(camera_frame);
      images_waiting = images_to_save_.size();
    }
    images_to_save_cv_.notify_one();
    if (static_cast<double>(images_waiting) > publish_hz_) {
      RCLCPP_WARN_THROTTLE(
        node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000,
        "%zu images waiting to be saved: the disk is not keeping up (raise save_threads?)",
        images_waiting);
    }
  }

  if (publish_images_) {
    camera_->updateData(frame);
  }

  output_queue_.push(camera_frame);
}

void UsbCameraInterface::saveImages()
{
  // Run by every writer thread. Each frame is taken from the queue under the lock, so it is saved
  // by exactly one writer; frames have unique stamps, so writers never share a file.
  std::unique_lock<std::mutex> lock(images_to_save_mutex_);
  while (true) {
    images_to_save_cv_.wait(lock, [this] {return stop_image_writers_ || !images_to_save_.empty();});
    if (images_to_save_.empty()) {
      return;  // Stopping, and every queued image is saved
    }
    const CameraFrame camera_frame = std::move(images_to_save_.front());
    images_to_save_.pop();
    lock.unlock();

    const std::string path = (std::filesystem::path(image_folder_) /
      (std::to_string(rclcpp::Time(camera_frame.header.stamp).nanoseconds()) + ".bmp")).string();
    if (!cv::imwrite(path, camera_frame.image)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Cannot save image %s", path.c_str());
    }
    lock.lock();
  }
}

}  // namespace usb_camera_interface
