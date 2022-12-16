/*!*******************************************************************************************
 *  \file       sensor.cpp
 *  \brief      Sensor class for AS2 implementation file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
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

#include "sensor.hpp"

namespace as2 {
namespace sensors {

Camera::Camera(const std::string &id, as2::Node *node_ptr) : GenericSensor(id, node_ptr) {
  camera_info_publisher_ =
      node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(topic_name_ + "/info", 10);
  camera_name_ = id;
}

Camera::~Camera(){};

void Camera::setup() {
  image_transport_ptr_ = std::make_shared<image_transport::ImageTransport>(getSelfPtr());
  image_transport::ImageTransport &image_transport_ = *image_transport_ptr_;

  it_publisher_ = image_transport_.advertise(topic_name_, 1);

  camera_frame_ = camera_name_ + "/" + camera_link_;
  camera_frame_ = as2::tf::generateTfName(node_ptr_->get_namespace(), camera_frame_);
  setup_        = false;
}

void Camera::updateData(const sensor_msgs::msg::Image &_img) {
  if (setup_) {
    setup();
  }

  if (pub_freq_ != -1) {
    image_data_ = _img;
  } else {
    publishCameraData(_img);
  }
}

void Camera::updateData(const cv::Mat &_img) {
  sensor_msgs::msg::Image img_msg;
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp    = node_ptr_->now();
  cv_image.header.frame_id = camera_frame_;
  cv_image.encoding        = encoding_;
  cv_image.image           = _img;
  cv_image.toImageMsg(img_msg);

  updateData(img_msg);
}

void Camera::publishCameraData(const sensor_msgs::msg::Image &_msg) {
  it_publisher_.publish(_msg);

  if (camera_info_available_) {
    camera_info_publisher_->publish(camera_info_data_);
  }
}

void Camera::publishRectifiedImage(const sensor_msgs::msg::Image &msg) {
  // Rectify image and publish it
  // image_publisher_->publish(msg);
}

// void Camera::publishCompressedImage(const sensor_msgs::msg::Image &msg)
// {
//   // Compress image and publish it
//   // image_publisher_->publish(msg);
// }

void Camera::loadParameters(const std::string &file) {
  // read configuration file
}

void Camera::setParameters(const sensor_msgs::msg::CameraInfo &_camera_info,
                           const std::string &_encoding,
                           const std::string &_camera_model) {
  camera_info_data_      = _camera_info;
  camera_info_available_ = true;

  encoding_     = _encoding;
  camera_model_ = _camera_model;
}

std::shared_ptr<rclcpp::Node> Camera::getSelfPtr() { return node_ptr_->shared_from_this(); }

}  // namespace sensors
};  // namespace as2
