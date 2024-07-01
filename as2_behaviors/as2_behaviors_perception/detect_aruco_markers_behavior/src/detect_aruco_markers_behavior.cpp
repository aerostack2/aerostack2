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

/*!*******************************************************************************************
 *  \file       detect_aruco_markers_behavior.cpp
 *  \brief      Aruco detector implementation file.
 *  \authors    David Perez Saura
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#include "detect_aruco_markers_behavior.hpp"

DetectArucoMarkersBehavior::DetectArucoMarkersBehavior()
: as2_behavior::BehaviorServer<as2_msgs::action::DetectArucoMarkers>(
    "detect_aruco_markers_behavior")
{
  loadParameters();
}

void DetectArucoMarkersBehavior::setup()
{
  aruco_pose_pub_ = this->create_publisher<as2_msgs::msg::PoseStampedWithID>(
    this->generate_local_name("aruco_pose"), rclcpp::SensorDataQoS());
  aruco_img_transport_ = std::make_shared<as2::sensors::Camera>(this, "aruco_img_topic");

  std::shared_ptr<const rclcpp::QoS> camera_qos;
  if (camera_qos_reliable_) {
    RCLCPP_INFO(get_logger(), "QoS Camera subscription: Reliable");
    camera_qos = std::make_shared<const rclcpp::QoS>(rclcpp::QoS(2));
  } else {
    RCLCPP_INFO(get_logger(), "QoS Camera subscription: Sensor");
    camera_qos = std::make_shared<const rclcpp::QoS>(rclcpp::SensorDataQoS());
  }

  cam_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    camera_image_topic_, *camera_qos,
    std::bind(&DetectArucoMarkersBehavior::imageCallback, this, std::placeholders::_1));

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, as2_names::topics::sensor_measurements::qos,
    std::bind(&DetectArucoMarkersBehavior::camerainfoCallback, this, std::placeholders::_1));
}

void DetectArucoMarkersBehavior::loadParameters()
{
  // std::string aruco_dictionary;
  this->declare_parameter<float>("aruco_size");
  this->declare_parameter<bool>("camera_qos_reliable");
  this->declare_parameter<std::string>("camera_image_topic");
  this->declare_parameter<std::string>("camera_info_topic");
  this->declare_parameter<std::string>("camera_model");

  this->get_parameter("aruco_size", aruco_size_);
  this->get_parameter("camera_qos_reliable", camera_qos_reliable_);
  this->get_parameter("camera_image_topic", camera_image_topic_);
  this->get_parameter("camera_info_topic", camera_info_topic_);
  this->get_parameter("camera_model", camera_model_);

  RCLCPP_INFO(get_logger(), "Params: aruco_size: %.3f m", aruco_size_);

  // TODO(david): Load dictionary from param
  aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  // aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
}

// SIMULATION
void DetectArucoMarkersBehavior::setCameraParameters(
  const sensor_msgs::msg::CameraInfo & _camera_info)
{
  camera_matrix_ = cv::Mat(3, 3, CV_64F);

  camera_matrix_.at<double>(0, 0) = _camera_info.k[0];
  camera_matrix_.at<double>(0, 1) = _camera_info.k[1];
  camera_matrix_.at<double>(0, 2) = _camera_info.k[2];
  camera_matrix_.at<double>(1, 0) = _camera_info.k[3];
  camera_matrix_.at<double>(1, 1) = _camera_info.k[4];
  camera_matrix_.at<double>(1, 2) = _camera_info.k[5];
  camera_matrix_.at<double>(2, 0) = _camera_info.k[6];
  camera_matrix_.at<double>(2, 1) = _camera_info.k[7];
  camera_matrix_.at<double>(2, 2) = _camera_info.k[8];

  int n_discoeff = _camera_info.d.size();
  dist_coeffs_ = cv::Mat(1, n_discoeff, CV_64F);

  for (int i; i < n_discoeff; i++) {
    dist_coeffs_.at<double>(0, i) = _camera_info.d[i];
  }

  // std::cout << dist_coeffs_ << std::endl;

  distorsion_model_ = _camera_info.distortion_model;

  if (camera_model_ == "fisheye") {
    RCLCPP_INFO(get_logger(), "Using FISHEYE camera model");
    if (n_discoeff != 4) {
      RCLCPP_ERROR(get_logger(), "FISHEYE distortion coefficients must be 4");
    }
  }
  if (camera_model_ == "pinhole") {
    RCLCPP_INFO(get_logger(), "Using PINHOLE camera model");
  }

  img_encoding_ = sensor_msgs::image_encodings::BGR8;

  camera_params_available_ = true;
}

void DetectArucoMarkersBehavior::camerainfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  RCLCPP_DEBUG(this->get_logger(), "Camera info received by callback");
  if (camera_params_available_) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Setting camera info");
  setCameraParameters(*info);
}

void DetectArucoMarkersBehavior::imageCallback(const sensor_msgs::msg::Image::SharedPtr img)
{
  RCLCPP_DEBUG(this->get_logger(), "Image received by callback");

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, img_encoding_);
  } catch (cv_bridge::Exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "CV_bridge exception: %s\n", ex.what());
  }

  if (!camera_params_available_) {
    RCLCPP_WARN(get_logger(), "No camera parameters available");
    return;
  }

  // init ArUco detection
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::aruco::DetectorParameters::create();

  // detect markers on the fisheye, it's no worth it to detect over the rectified image
  cv::aruco::detectMarkers(
    cv_ptr->image, aruco_dict_, marker_corners, marker_ids, detector_params, rejected_candidates);
  cv::Mat output_image = cv_ptr->image.clone();
  cv::Vec3d aruco_position, aruco_rotation;

  if (marker_ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(
      output_image, marker_corners,
      marker_ids);  // Draw markers to ensure orientation
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
      marker_corners, aruco_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    for (int i = 0; i < rvecs.size(); ++i) {
      int id = marker_ids[i];

      if (!checkIdIsTarget(id)) {
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Marker %d detected", id);
      cv::Vec3d rvec = rvecs[i];
      cv::Vec3d tvec = tvecs[i];
      cv::Vec3d rout, tout;
      cv::composeRT(rvec * 0, tvec * 0, rvec, tvec, rout, tout);
      aruco_position = tout;
      aruco_rotation = rout;
      cv::drawFrameAxes(output_image, camera_matrix_, dist_coeffs_, rout, tout, 0.08625, 3);
    }
  }

  cv::Mat undistort_camera_matrix;
  cv::Mat rectified_image, cropped_image;
  cv::Rect roi;
  float alpha = 0.0;

  if (camera_model_ == "pinhole") {
    RCLCPP_INFO_ONCE(get_logger(), "Undistort image with pinhole model");
    cv::undistort(output_image, rectified_image, camera_matrix_, dist_coeffs_);
  }

  if (camera_model_ == "fisheye") {
    RCLCPP_INFO_ONCE(get_logger(), "Undistort image with fisheye model");
    cv::fisheye::undistortImage(output_image, rectified_image, camera_matrix_, dist_coeffs_);
  }

  std::string output_img_encoding = sensor_msgs::image_encodings::BGR8;

  sensor_msgs::msg::Image output_image_msg =
    *(cv_bridge::CvImage(std_msgs::msg::Header(), output_img_encoding, rectified_image)
    .toImageMsg()
    .get());
  aruco_img_transport_->updateData(output_image_msg);

  for (int i = 0; i < marker_ids.size(); i++) {
    int id = marker_ids[i];
    if (checkIdIsTarget(id)) {
      // std::cout << aruco_position << std::endl;
      as2_msgs::msg::PoseStampedWithID pose;
      pose.pose.header = img->header;
      pose.id = std::to_string(id);
      pose.pose.pose.position.x = aruco_position[0];
      pose.pose.pose.position.y = aruco_position[1];
      pose.pose.pose.position.z = aruco_position[2];
      tf2::Quaternion rot;
      rot.setRPY(aruco_rotation[2], aruco_rotation[0], aruco_rotation[1]);
      rot = rot.normalize();
      pose.pose.pose.orientation.x = static_cast<double>(rot.x());
      pose.pose.pose.orientation.y = static_cast<double>(rot.y());
      pose.pose.pose.orientation.z = static_cast<double>(rot.z());
      pose.pose.pose.orientation.w = static_cast<double>(rot.w());

      aruco_pose_pub_->publish(pose);
    }
  }
}

bool DetectArucoMarkersBehavior::checkIdIsTarget(const int _id)
{
  // if target_ids_ is not empty
  if (target_ids_.size() > 0) {
    // if id is not in target_ids_
    if (std::find(target_ids_.begin(), target_ids_.end(), _id) == target_ids_.end()) {
      return false;
    }
  }
  return true;
}

//** AS2 Behavior methods **//
bool DetectArucoMarkersBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::DetectArucoMarkers::Goal> goal)
{
  setup();
  target_ids_ = goal->target_ids;
  RCLCPP_INFO(get_logger(), "Goal accepted");
  RCLCPP_INFO(get_logger(), "Target IDs: %s", targetIds2string(target_ids_).c_str());
  return true;
}

bool DetectArucoMarkersBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::DetectArucoMarkers::Goal> goal)
{
  target_ids_ = goal->target_ids;
  RCLCPP_INFO(get_logger(), "Goal modified");
  RCLCPP_INFO(get_logger(), "New target IDs: %s", targetIds2string(target_ids_).c_str());
  return true;
}

bool DetectArucoMarkersBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(get_logger(), "DetectArucoMarkersBehavior cancelled");
  return true;
}

bool DetectArucoMarkersBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(get_logger(), "DetectArucoMarkersBehavior paused");
  return true;
}

bool DetectArucoMarkersBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(get_logger(), "DetectArucoMarkersBehavior resumed");
  return true;
}

as2_behavior::ExecutionStatus DetectArucoMarkersBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::DetectArucoMarkers::Goal> & goal,
  std::shared_ptr<as2_msgs::action::DetectArucoMarkers::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::DetectArucoMarkers::Result> & result_msg)
{
  return as2_behavior::ExecutionStatus::RUNNING;
}

void DetectArucoMarkersBehavior::on_execution_end(const as2_behavior::ExecutionStatus & status)
{
  cam_image_sub_.reset();
  cam_info_sub_.reset();
  aruco_pose_pub_.reset();
  aruco_img_transport_.reset();
  target_ids_.clear();

  RCLCPP_INFO(get_logger(), "DetectArucoMarkersBehavior execution ended");
  return;
}

std::string targetIds2string(const std::vector<uint16_t> & target_ids)
{
  int max_id_show = 10;
  std::string target_ids_str = "";
  if (target_ids.size() == 0) {
    return "All";
  }
  if (target_ids.size() < max_id_show) {
    for (int i = 0; i < target_ids.size(); i++) {
      target_ids_str += std::to_string(target_ids[i]);
      if (i != target_ids.size() - 1) {
        target_ids_str += ", ";
      }
    }
  } else {
    target_ids_str = std::to_string(target_ids[0]);
    target_ids_str += ", ... , ";
    target_ids_str += std::to_string(target_ids[target_ids.size() - 1]);
  }
  return target_ids_str;
}
