// Copyright 2023 Universidad Politécnica de Madrid
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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
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

/*!*******************************************************************************************
 *  \file       dji_camera_handler.hpp
 *  \brief      DJI Camera Handler class header file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#ifndef DJI_CAMERA_HANDLER_HPP_
#define DJI_CAMERA_HANDLER_HPP_

#include <string>

// dji includes
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_msgs/msg/gimbal_control.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
// #include "dji_liveview.hpp"
#include "dji_vehicle.hpp"
#include "dji_waypoint_v2_action.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "osdk_platform.h"
#include "osdkhal_linux.h"

#include "dji_advanced_sensing.hpp"
#include "dji_gimbal_manager.hpp"
#include "dji_linux_helpers.hpp"

using namespace DJI::OSDK;

void camera_cb(CameraRGBImage pImg, void* userData);
void main_camera_cb(CameraRGBImage pImg, void* userData);

class DJIGimbalHandler {
  DJI::OSDK::Vehicle* vehicle_ptr_;
  as2::Node* node_ptr_;
  DJI::OSDK::GimbalManager gimbal_manager_;
  rclcpp::Subscription<as2_msgs::msg::GimbalControl>::SharedPtr gimbal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr
      gimbal_status_pub_;
  geometry_msgs::msg::Vector3 gimbal_angle_;

 public:
  DJIGimbalHandler(DJI::OSDK::Vehicle* vehicle, as2::Node* node)
      : vehicle_ptr_(vehicle), node_ptr_(node), gimbal_manager_(vehicle) {
    // activate gimbal
    std::string gimbal_name = "gimbal";

    gimbal_manager_.initGimbalModule(DJI::OSDK::PAYLOAD_INDEX_0,
                                     gimbal_name.c_str());

    gimbal_sub_ = node_ptr_->create_subscription<as2_msgs::msg::GimbalControl>(
        "platform/" + gimbal_name + "/gimbal_command", 10,
        std::bind(&DJIGimbalHandler::gimbalCb, this, std::placeholders::_1));

    gimbal_status_pub_ =
        node_ptr_->create_publisher<geometry_msgs::msg::QuaternionStamped>(
            "sensor_measurements/" + gimbal_name + "/attitude",
            as2_names::topics::sensor_measurements::qos);

    gimbal_angle_.x = 0;
    gimbal_angle_.y = 0;
    gimbal_angle_.z = 0;
  }

  void gimbalCb(const as2_msgs::msg::GimbalControl::SharedPtr msg) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Gimbal angle: %f %f %f",
                msg->target.vector.x, msg->target.vector.y,
                msg->target.vector.z);
    DJI::OSDK::GimbalModule::Rotation gimbal_rotation;
    if (msg->target.vector.x == gimbal_angle_.x &&
        msg->target.vector.y == gimbal_angle_.y &&
        msg->target.vector.z == gimbal_angle_.z) {
      return;
    }
    gimbal_rotation.roll = msg->target.vector.x - gimbal_angle_.x;
    gimbal_rotation.pitch = msg->target.vector.y - gimbal_angle_.y;
    gimbal_rotation.yaw = msg->target.vector.z - gimbal_angle_.z;
    gimbal_angle_.x = msg->target.vector.x;
    gimbal_angle_.y = msg->target.vector.y;
    gimbal_angle_.z = msg->target.vector.z;
    gimbal_rotation.time = 0.5;
    gimbal_rotation.rotationMode = 0;
    ErrorCode::ErrorCodeType error = gimbal_manager_.rotateSync(
        DJI::OSDK::PAYLOAD_INDEX_0, gimbal_rotation, 1);
    if (error != ErrorCode::SysCommonErr::Success) {
      ErrorCode::printErrorCodeMsg(error);
    } else {
      geometry_msgs::msg::QuaternionStamped gimbal_status;
      gimbal_status.header.stamp = node_ptr_->now();
      gimbal_status.header.frame_id = "base_link";
      as2::frame::eulerToQuaternion(gimbal_angle_.x, gimbal_angle_.y,
                                    gimbal_angle_.z, gimbal_status.quaternion);
      gimbal_status_pub_->publish(gimbal_status);
    }
  }
};

class DJICameraTrigger {
  DJI::OSDK::Vehicle* vehicle_ptr_;
  as2::Node* node_ptr_;
  DJI::OSDK::CameraManager camera_manager_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;

 public:
  DJICameraTrigger(DJI::OSDK::Vehicle* vehicle, as2::Node* node)
      : vehicle_ptr_(vehicle), node_ptr_(node), camera_manager_(vehicle) {
    camera_manager_.initCameraModule(DJI::OSDK::PAYLOAD_INDEX_0, "camera");

    trigger_sub_ = node_ptr_->create_subscription<std_msgs::msg::Bool>(
        "camera/trigger", 10,
        std::bind(&DJICameraTrigger::triggerCb, this, std::placeholders::_1));
  }

  void triggerCb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Triggering camera");
      camera_manager_.startShootPhotoSync(DJI::OSDK::PAYLOAD_INDEX_0,
                                          DJI::OSDK::CameraModule::SINGLE, 2);
      // DJI::OSDK::PAYLOAD_INDEX_0, DJI::OSDK::CameraModule::REGIONAL_SR, 2);
    }
  }
};

class DJICameraHandler {
  DJI::OSDK::Vehicle* vehicle_ptr_;
  as2::Node* node_ptr_;
  as2::sensors::Camera camera_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr camera_source_sub_;

 public:
  DJICameraHandler(DJI::OSDK::Vehicle* vehicle, as2::Node* node)
      : vehicle_ptr_(vehicle), node_ptr_(node), camera_("image_raw", node) {
    camera_.setParameters(sensor_msgs::msg::CameraInfo(),
                          sensor_msgs::image_encodings::BGR8, std::string(""));

    camera_source_sub_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>(
        "camera/source", 10,
        std::bind(&DJICameraHandler::change_camera_source, this,
                  std::placeholders::_1));
  }

  void start_camera() {
    if (vehicle_ptr_->getFwVersion() < Version::M100_31) {
      std::cout << "Camera is not supported on this platform" << std::endl;
      return;
    }

    // vehicle_ptr_->advancedSensing->startMainCameraStream(camera_cb,nullptr);
    /* if(vehicle_ptr_->advancedSensing->startFPVCameraStream(camera_cb,nullptr)){
      std::cout << "FPV camera stream started" << std::endl;
    }
    else{
      std::cout << "FPV camera stream failed to start" << std::endl;
    } */
    if (vehicle_ptr_->advancedSensing->startMainCameraStream(main_camera_cb,
                                                             (void*)&camera_)) {
      std::cout << "Main camera stream started" << std::endl;

    } else {
      std::cout << "Main camera stream failed to start" << std::endl;
    }

    // AdvancedSensing::startH264Stream(LiveView::LiveViewCameraPosition pos,
    // H264Callback cb, void *userData);
  }
  void stop_camera() {
    vehicle_ptr_->advancedSensing->stopFPVCameraStream();
    vehicle_ptr_->advancedSensing->stopMainCameraStream();
  }

  void change_camera_source(const std_msgs::msg::UInt8::SharedPtr msg) {
    // LiveView::LiveViewCameraPosition
    // 0: DJI::OSDK::LiveView::LiveViewCameraPosition::OSDK_CAMERA_POSITION_NO_1
    // 1: DJI::OSDK::LiveView::LiveViewCameraPosition::OSDK_CAMERA_POSITION_NO_2
    // 2: DJI::OSDK::LiveView::LiveViewCameraPosition::OSDK_CAMERA_POSITION_NO_3
    // 7: DJI::OSDK::LiveView::LiveViewCameraPosition::OSDK_CAMERA_POSITION_FPV

    vehicle_ptr_->advancedSensing->changeH264Source(
        LiveView::LiveViewCameraPosition::OSDK_CAMERA_POSITION_NO_1,
        static_cast<LiveView::LiveViewCameraSource>(msg->data));
  }
};

#endif  // DJI_CAMERA_HANDLER_HPP_
