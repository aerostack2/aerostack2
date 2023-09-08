/*!*******************************************************************************************
 *  \file       as2_realsense_interface.hpp
 *  \brief      Realsense camera interface header file.
 *  \authors    David Perez Saura
 *  \copyright  Copyright (c) 2021 Universidad Politécnica de Madrid
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

#ifndef AS2_REALSENSE_INTERFACE_HPP_
#define AS2_REALSENSE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"

#include <tf2_ros/static_transform_broadcaster.h>
#include <librealsense2/rs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

class RealsenseInterface : public as2::Node {
public:
  /**
   * @brief Constructor of the RealsenseInterface object
   *
   */
  RealsenseInterface();

  /**
   * @brief Stop rutine for odometry node.
   * This function stops the pipeline.
   */
  void stop();

  /**
   * @brief Funtionality during node lifetime.
   * This function gets the sensor measurements
   * from realsense device and publish them.
   */
  void run();

private:
  std::string realsense_name_;

  bool verbose_;
  bool device_not_found_;
  bool imu_available_;
  bool depth_available_;
  bool color_available_;
  bool fisheye_available_;
  bool pose_available_;

  // Sensor comm
  std::string serial_;
  rs2::pipeline pipe_;
  // Sensor measurement
  std::shared_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> pose_sensor_;
  std::shared_ptr<as2::sensors::Imu> imu_sensor_;
  std::shared_ptr<as2::sensors::Camera> color_sensor_;
  std::shared_ptr<rs2::motion_frame> accel_frame_;
  std::shared_ptr<rs2::motion_frame> gyro_frame_;
  std::shared_ptr<rs2::pose_frame> pose_frame_;
  std::shared_ptr<rs2::video_frame> color_frame_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  std::vector<geometry_msgs::msg::TransformStamped> tf2_fix_transforms_;
  geometry_msgs::msg::TransformStamped rs_odom2rs_link_tf_;

  std::string base_link_frame_;
  std::string odom_frame_;
  std::string realsense_link_frame_;

  std::string color_sensor_frame_;
  std::string imu_sensor_frame_;

  tf2::Transform base_link_to_realsense_link_;
  tf2::Transform base_link_to_realsense_pose_odom_;
  tf2::Transform realsense_link_to_realsense_pose_;
  tf2::Transform realsense_link_to_realsense_pose_odom_;
  tf2::Transform realsense_pose_;

  bool setup();

  void setupCamera(const std::shared_ptr<as2::sensors::Camera> &_camera,
                   const rs2_stream _rs2_stream,
                   const std::string _encoding,
                   const std::string _camera_model);

  void runImu(const rs2::motion_frame &accel_frame, const rs2::motion_frame &gyro_frame);
  void runPose(const rs2::pose_frame &pose_frame);
  void runOdom(const rs2::pose_frame &pose_frame);
  void runColor(const rs2::video_frame &color_frame);

  bool identifyDevice();
  bool identifySensors(const rs2::device &dev);
  void setStaticTransform(const std::string _rs_link,
                          const std::string _ref_frame,
                          const std::array<double, 3> &_t,
                          const std::array<double, 3> &_r);

  void setupPoseTransforms(const std::array<double, 3> &device_t,
                           const std::array<double, 3> &device_r);
};

#endif  // AS2_REALSENSE_INTERFACE_HPP_
