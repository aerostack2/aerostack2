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
* @file as2_usb_camera_interface.hpp
*
* @brief Class definition for the generic camera interface
*
* @details Generic, reusable camera driver supporting any OpenCV-readable
* device (USB / V4L2) as well as the Jetson CSI Arducam (via GStreamer).
* It is built around an as2::Node pointer so it can either run as a
* standalone driver node or be embedded inside another node (e.g. a
* perception behavior) that consumes the captured frames from the output
* queue.
*
* @authors David Perez Saura, Miguel Fernandez Cortizas
*/

#ifndef AS2_USB_CAMERA_INTERFACE__AS2_USB_CAMERA_INTERFACE_HPP_
#define AS2_USB_CAMERA_INTERFACE__AS2_USB_CAMERA_INTERFACE_HPP_

#include <cv_bridge/cv_bridge.h>

#include <cstring>
#include <array>
#include <vector>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"

#include <opencv2/opencv.hpp>

#include "as2_usb_camera_interface/common.hpp"


namespace usb_camera_interface
{

/**
 * @brief A single captured frame together with its header.
 */
struct CameraFrame
{
  std_msgs::msg::Header header;
  cv::Mat image;
};

class UsbCameraInterface
{
public:
  /**
   * @brief Construct a new UsbCameraInterface object bound to an existing node.
   *
   * @param node_ptr Node used to read parameters, create the capture timer and
   *                 publish the image / camera_info / TF.
   */
  explicit UsbCameraInterface(as2::Node * node_ptr);

  /**
   * @brief Destroy the UsbCameraInterface object
   */
  ~UsbCameraInterface() = default;

  /**
   * @brief Get the latest known camera info (intrinsics, distortion, ...).
   */
  sensor_msgs::msg::CameraInfo getCameraInfoMessage() const {return camera_info_;}

  /**
   * @brief Thread-safe queue holding the captured frames for in-process consumers.
   */
  as2_usb_camera_interface::MutexQueue<CameraFrame> & getOutputQueue() {return output_queue_;}

private:
  as2::Node * node_ptr_;
  std::shared_ptr<as2::sensors::Camera> camera_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr image_capture_timer_;
  rclcpp::CallbackGroup::SharedPtr capture_callback_group_;

  as2_usb_camera_interface::MutexQueue<CameraFrame> output_queue_{1};
  sensor_msgs::msg::CameraInfo camera_info_;

  std::string camera_name_;
  double framerate_{30.0};
  bool publish_images_{true};

  void captureImage();
  void setupCamera();
  void cameraInfoSetup();

  template<std::size_t N>
  bool convertVectorToArray(const std::vector<double> & vec, std::array<double, N> & array)
  {
    if (vec.size() != array.size()) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "The vector size is different from the array size. Vector size: %ld, Array size: %ld",
        vec.size(), array.size());
      return false;
    }
    std::memcpy(array.data(), vec.data(), array.size() * sizeof(double));
    return true;
  }
};

/**
 * @brief Thin standalone driver node that owns a UsbCameraInterface.
 *
 * Used by the standalone executable; the UsbCameraInterface itself is meant to
 * be embedded in any as2::Node.
 */
class UsbCameraInterfaceNode : public as2::Node
{
public:
  explicit UsbCameraInterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : as2::Node("usb_camera_interface", options)
  {
    camera_interface_ = std::make_shared<UsbCameraInterface>(this);
  }

private:
  std::shared_ptr<UsbCameraInterface> camera_interface_;
};

}  // namespace usb_camera_interface

#endif  // AS2_USB_CAMERA_INTERFACE__AS2_USB_CAMERA_INTERFACE_HPP_
