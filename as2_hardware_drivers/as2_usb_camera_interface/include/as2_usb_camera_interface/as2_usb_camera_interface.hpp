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
* @brief Class definition for the USB camera interface node
*
* @authors David Perez Saura, Miguel Fernandez Cortizas
*/

#ifndef AS2_USB_CAMERA_INTERFACE__AS2_USB_CAMERA_INTERFACE_HPP_
#define AS2_USB_CAMERA_INTERFACE__AS2_USB_CAMERA_INTERFACE_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"


#include <image_transport/image_transport.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>


namespace usb_camera_interface
{

class UsbCameraInterface : public as2::Node
{
public:
  /**
   * @brief Construct a new UsbCameraInterface object
   */
  explicit UsbCameraInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the UsbCameraInterface object
   */
  ~UsbCameraInterface() {}

private:
  std::shared_ptr<as2::sensors::Camera> camera_;

  std::string device_port_;
  int image_width_;
  int image_height_;
  double framerate_;

  cv::VideoCapture cap_;

  void loadParameters();
  void captureImage();
  void setupCamera();
};

}  // namespace usb_camera_interface

#endif  // AS2_USB_CAMERA_INTERFACE__AS2_USB_CAMERA_INTERFACE_HPP_
