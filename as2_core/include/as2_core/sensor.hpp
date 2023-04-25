/*!*******************************************************************************************
 *  \file       sensor.hpp
 *  \brief      Sensor class for AS2 header file
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

#ifndef AEROSTACK2_SENSOR_HPP_
#define AEROSTACK2_SENSOR_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "aerial_platform.hpp"
#include "as2_core/names/topics.hpp"
#include "node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/range.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "utils/tf_utils.hpp"

// camera
#include <image_transport/image_transport.hpp>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace as2 {
namespace sensors {
class GenericSensor {
public:
  GenericSensor(const std::string &topic_name, as2::Node *node_ptr, int pub_freq = -1)
      : node_ptr_(node_ptr), pub_freq_(pub_freq) {
    // check if topic already has "sensor_measurements "in the name
    // if not, add it
    if (topic_name.find(as2_names::topics::sensor_measurements::base) == std::string::npos) {
      topic_name_ = as2_names::topics::sensor_measurements::base + topic_name;
    } else {
      topic_name_ = topic_name;
    }
  }

protected:
  std::string topic_name_;
  as2::Node *node_ptr_ = nullptr;
  float pub_freq_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  // std::string sensor_id_;

  virtual void setStaticTransform(const std::string &frame_id,
                                  const std::string &parent_frame_id,
                                  float x,
                                  float y,
                                  float z,
                                  float qx,
                                  float qy,
                                  float qz,
                                  float qw) {
    setStaticTransform_(frame_id, parent_frame_id, x, y, z, qx, qy, qz, qw);
  }

  virtual void setStaticTransform(const std::string &frame_id,
                                  const std::string &parent_frame_id,
                                  float x,
                                  float y,
                                  float z,
                                  float roll,
                                  float pitch,
                                  float yaw) {
    // convert to quaternion and set the transform
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    setStaticTransform_(frame_id, parent_frame_id, x, y, z, q.x(), q.y(), q.z(), q.w());
  };

protected:
  void setStaticTransform_(const std::string &frame_id,
                           const std::string &parent_frame_id,
                           float x,
                           float y,
                           float z,
                           float qx,
                           float qy,
                           float qz,
                           float qw) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster((rclcpp::Node *)node_ptr_);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp            = rclcpp::Clock().now();
    transformStamped.header.frame_id         = parent_frame_id;
    transformStamped.child_frame_id          = frame_id;
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;
    transformStamped.transform.rotation.x    = qx;
    transformStamped.transform.rotation.y    = qy;
    transformStamped.transform.rotation.z    = qz;
    transformStamped.transform.rotation.w    = qw;
    static_broadcaster.sendTransform(transformStamped);
    RCLCPP_INFO(node_ptr_->get_logger(), "Static transform published: %s -> %s",
                parent_frame_id.c_str(), frame_id.c_str());
  }

  void registerSensor(){};

};  // class GenericSensor

template <typename T>
class Sensor : public GenericSensor {
public:
  Sensor(const std::string &id, as2::Node *node_ptr, int pub_freq = -1)
      : GenericSensor(id, node_ptr, pub_freq) {
    sensor_publisher_ = node_ptr_->create_publisher<T>(this->topic_name_,
                                                       as2_names::topics::sensor_measurements::qos);

    if (this->pub_freq_ != -1) {
      timer_ = node_ptr_->create_timer(std::chrono::milliseconds(1000 / pub_freq),
                                       [this]() { publishData(); });
    }
  }

  void updateData(const T &msg) {
    if (this->pub_freq_ != -1) {
      this->msg_data_ = msg;
    } else {
      this->publishData(msg);
    }
  }

private:
  void publishData() { this->sensor_publisher_->publish(msg_data_); }
  void publishData(const T &msg) { this->sensor_publisher_->publish(msg); }

private:
  typename rclcpp::Publisher<T>::SharedPtr sensor_publisher_;
  T msg_data_;

};  // class Sensor

class Camera : public GenericSensor {
public:
  Camera(const std::string &id, as2::Node *node_ptr);
  ~Camera();

  void updateData(const sensor_msgs::msg::Image &_img);
  void updateData(const cv::Mat &_img);
  void loadParameters(const std::string &file);
  void setParameters(const sensor_msgs::msg::CameraInfo &_camera_info,
                     const std::string &_encoding,
                     const std::string &_camera_model = "pinhole");

  void publishRectifiedImage(const sensor_msgs::msg::Image &msg);
  // void publishCompressedImage(const sensor_msgs::msg::Image &msg);

  void setStaticTransform(const std::string &frame_id,
                          const std::string &parent_frame_id,
                          float x,
                          float y,
                          float z,
                          float qx,
                          float qy,
                          float qz,
                          float qw) override {
    // FIXME: enhance performance
    // obtain r,p and yaw from quaternion
    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    setStaticTransform(frame_id, parent_frame_id, x, y, z, roll, pitch, yaw);
  }

  void setStaticTransform(const std::string &frame_id,
                          const std::string &parent_frame_id,
                          float x,
                          float y,
                          float z,
                          float roll,
                          float pitch,
                          float yaw) override {
    // convert to quaternion and set the transform
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    setStaticTransform_(frame_id, parent_frame_id, x, y, z, q.x(), q.y(), q.z(), q.w());

    // if frame_id does not contain "camera_link"
    if (frame_id.find(camera_link_) == std::string::npos) {
      // set the static transform for the camera_link frame rotating from FLU  to RDF
      yaw   = -M_PI / 2.0f;
      pitch = 0;
      roll  = -M_PI / 2.0f;
      q.setRPY(roll, pitch, yaw);
      setStaticTransform_(frame_id + "/" + camera_link_, frame_id, 0, 0, 0, q.x(), q.y(), q.z(),
                          q.w());
    }
  };

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_ptr_ = nullptr;
  image_transport::Publisher it_publisher_;

  sensor_msgs::msg::Image image_data_;
  sensor_msgs::msg::CameraInfo camera_info_data_;

  bool setup_                 = true;
  bool camera_info_available_ = false;

  std::string camera_name_;
  std::string camera_link_ = "camera_link";
  std::string camera_frame_;
  std::string encoding_;
  std::string camera_model_;

  std::shared_ptr<rclcpp::Node> getSelfPtr();
  void setup();
  void publishCameraData(const sensor_msgs::msg::Image &msg);
};  // class CameraSensor

using Imu         = Sensor<sensor_msgs::msg::Imu>;
using GPS         = Sensor<sensor_msgs::msg::NavSatFix>;
using Lidar       = Sensor<sensor_msgs::msg::LaserScan>;
using Battery     = Sensor<sensor_msgs::msg::BatteryState>;
using Barometer   = Sensor<sensor_msgs::msg::FluidPressure>;
using Compass     = Sensor<sensor_msgs::msg::MagneticField>;
using RangeFinder = Sensor<sensor_msgs::msg::Range>;

};  // namespace sensors
};  // namespace as2

#endif  // AEROSTACK2_NODE_HPP_
