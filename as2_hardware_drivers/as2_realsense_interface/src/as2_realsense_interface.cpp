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
* @file as2_realsense_interface.cpp
*
* as2_realsense_interface source file.
*
* @author David Perez Saura
*/


#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <iostream>
#include <librealsense2/h/rs_sensor.h>
#include <vector>
#include <string>
#include <memory>

#include "as2_realsense_interface/as2_realsense_interface.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <rclcpp/callback_group.hpp>

namespace real_sense_interface
{

RealsenseInterface::RealsenseInterface(const rclcpp::NodeOptions & options)
: as2::Node("realsense_interface", options)
{
  // Initialize the transform broadcaster
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  std::string ns = this->get_namespace();
  base_link_frame_ = as2::tf::generateTfName(ns, "base_link");
  odom_frame_ = as2::tf::generateTfName(ns, "odom");

  setup();
}

bool RealsenseInterface::setup()
{
  if (!identifyDevice()) {
    RCLCPP_ERROR(this->get_logger(), "No device found");
    device_not_found_ = true;
    return false;
  }

  // Get parameters
  this->get_parameter("rs_name", realsense_name_);
  verbose_ = this->declare_parameter("verbose", true);

  std::string tf_link_frame;
  std::string tf_ref_frame;
  std::array<double, 3> tf_translation;
  std::array<double, 3> tf_rotation;

  this->declare_parameter<std::string>("tf_device.frame_id", "realsense_link");
  this->declare_parameter<std::string>("tf_device.reference_frame");
  this->declare_parameter<double>("tf_device.x");
  this->declare_parameter<double>("tf_device.y");
  this->declare_parameter<double>("tf_device.z");
  this->declare_parameter<double>("tf_device.roll");
  this->declare_parameter<double>("tf_device.pitch");
  this->declare_parameter<double>("tf_device.yaw");
  this->declare_parameter("undistort_image", false);
  this->declare_parameter<bool>("publish_images", false);
  this->declare_parameter<bool>("publish_image_1", false);
  this->declare_parameter<bool>("publish_image_2", false);

  // tf
  this->get_parameter("tf_device.frame_id", tf_link_frame);
  this->get_parameter("tf_device.reference_frame", tf_ref_frame);
  this->get_parameter("tf_device.x", tf_translation[0]);
  this->get_parameter("tf_device.y", tf_translation[1]);
  this->get_parameter("tf_device.z", tf_translation[2]);
  this->get_parameter("tf_device.roll", tf_rotation[0]);
  this->get_parameter("tf_device.pitch", tf_rotation[1]);
  this->get_parameter("tf_device.yaw", tf_rotation[2]);
  this->get_parameter("undistort_image", undistort_image_);
  this->get_parameter("publish_images", publish_images_);
  this->get_parameter("publish_image_1", publish_image_1_);
  this->get_parameter("publish_image_2", publish_image_2_);

  RCLCPP_INFO(this->get_logger(), "Publishing images: %s", (publish_images_ ? "true" : "false"));
  RCLCPP_INFO(this->get_logger(), "Publishing image 1: %s", (publish_image_1_ ? "true" : "false"));
  RCLCPP_INFO(this->get_logger(), "Publishing image 2: %s", (publish_image_2_ ? "true" : "false"));

  tf_ref_frame = as2::tf::generateTfName(this->get_namespace(), tf_ref_frame);
  realsense_link_frame_ = as2::tf::generateTfName(this->get_namespace(), tf_link_frame);
  imu_sensor_frame_ = realsense_link_frame_ + "/imu";

  // Publishers
  pose_sensor_ =
    std::make_shared<as2::sensors::Sensor<nav_msgs::msg::Odometry>>(tf_link_frame + "/odom", this);
  imu_sensor_ = std::make_shared<as2::sensors::Imu>(tf_link_frame + "/imu", this);
  color_sensor_ = std::make_shared<as2::sensors::Camera>(this, tf_link_frame + "/color");

  fisheye1_sensor_ = std::make_shared<as2::sensors::Camera>(this, tf_link_frame + "/fisheye1");
  fisheye2_sensor_ = std::make_shared<as2::sensors::Camera>(this, tf_link_frame + "/fisheye2");


  // Publish device static transform
  setStaticTransform(realsense_link_frame_, tf_ref_frame, tf_translation, tf_rotation);

  // Extract translation
  std::array<double, 3> RUB_translation = {0.0, 0.0091, 0.0};
  std::array<double, 3> RUB_rotation = {M_PI_2, 0.0, -M_PI_2};
  pose_link_frame_ = realsense_link_frame_ + "/pose";
  setStaticTransform(pose_link_frame_, realsense_link_frame_, RUB_translation, RUB_rotation);

  realsense_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  odom_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  realsense_frame_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1), std::bind(
      &RealsenseInterface::run,
      this), realsense_callback_group_);

  pose_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(5), std::bind(&RealsenseInterface::runPoseCallback, this),
    odom_callback_group_);

  if (publish_images_) {
    if (publish_image_1_) {
      fisheye1_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      fisheye1_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&RealsenseInterface::runFisheye1Callback, this),
        fisheye1_callback_group_);
    }
    if (publish_image_2_) {
      fisheye2_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      fisheye2_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&RealsenseInterface::runFisheye2Callback, this),
        fisheye2_callback_group_);
    }
  }


  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;
  if (!serial_.empty()) {
    cfg.enable_device(serial_);
  }

  if (imu_available_) {
	std::cout << "Enable IMU streams" << std::endl;
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
  }

  if (pose_available_) {
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  }

  /*
  if (color_available_) {
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
  }

  if (depth_available_) {
    cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);
  }*/

  if (publish_images_ && fisheye_available_) {
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
  }

  // There is the option to enable all streams
  // cfg.enable_all_streams();

  // Start pipeline with chosen configuration
  pipe_.start(cfg);

  if (color_available_) {
    RCLCPP_INFO(this->get_logger(), "Configuring color stream");

    std::string encoding = sensor_msgs::image_encodings::RGB8;
    color_sensor_frame_ = realsense_link_frame_ + "/color";
    setupCamera(color_sensor_, RS2_STREAM_COLOR, 1, encoding, color_sensor_frame_);
  }

  if (pose_available_) {
    setupPoseTransforms(tf_translation, tf_rotation);
  }

  if (publish_images_ && fisheye_available_) {
    RCLCPP_INFO(this->get_logger(), "Configuring fisheye streams");

    std::string encoding = sensor_msgs::image_encodings::MONO8;
    if (publish_image_1_) {
      std::cout << "Setup Fisheye 1" << std::endl;
      fisheye1_sensor_frame_ = realsense_link_frame_ + "/fisheye1/camera_link";
      setupCamera(fisheye1_sensor_, RS2_STREAM_FISHEYE, 1, encoding, fisheye1_sensor_frame_);
    }
    if (publish_image_2_) {
      fisheye2_sensor_frame_ = realsense_link_frame_ + "/fisheye2/camera_link";
      std::cout << "Setup Fisheye 2" << std::endl;
      setupCamera(fisheye2_sensor_, RS2_STREAM_FISHEYE, 2, encoding, fisheye2_sensor_frame_);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Realsense device node ready");

  return true;
}

void RealsenseInterface::stop() {pipe_.stop();}

void RealsenseInterface::run()
{
  if (device_not_found_) {
    // TODO(david): Wait for device to be connected
    RCLCPP_ERROR_ONCE(this->get_logger(), "Waiting for restart ...");
    return;
  }
  RCLCPP_INFO_ONCE(this->get_logger(), "Device running");

  // Wait for the next set of frames from the camera
  auto frames = pipe_.wait_for_frames();

  // Get data frames from device
  for (auto frame : frames) {
    // IMU
    if (frame.get_profile().stream_type() == RS2_STREAM_ACCEL) {
      accel_frame_ = std::make_shared<rs2::motion_frame>(frame.as<rs2::motion_frame>());
    }

    if (frame.get_profile().stream_type() == RS2_STREAM_GYRO) {
      gyro_frame_ = std::make_shared<rs2::motion_frame>(frame.as<rs2::motion_frame>());
    }
    // D435(i) RGB IMAGE
    // if (frame.get_profile().stream_type() == RS2_STREAM_COLOR) {
    //   color_frame_ = std::make_shared<rs2::video_frame>(frame.as<rs2::video_frame>());
    // }

    // T265 POSE
    if (frame.get_profile().stream_type() == RS2_STREAM_POSE) {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      pose_frame_ = std::make_shared<rs2::pose_frame>(frame.as<rs2::pose_frame>());
    }

    // D435(i) DEPTH
    // if (frame.get_profile().stream_type() == RS2_STREAM_DEPTH) {
    //   // TODO(david): Implement depth frame for D435(i)
    //   // auto depth       = frame.as<rs2::video_frame>();
    //   // auto depth_data  = depth.get_data();
    // }

    // T265 FISHEYE
    if (frame.get_profile().stream_type() == RS2_STREAM_FISHEYE) {
      if (!fisheye_available_) {
        continue;
      }
      auto local_frame = frame.as<rs2::video_frame>();
      auto fisheye_index = local_frame.get_profile().stream_index();
      if (fisheye_index == 1 && publish_image_1_) {
        fisheye_1_frame_ = std::make_shared<rs2::video_frame>(local_frame);
      } else if (fisheye_index == 2 && publish_image_2_) {
        fisheye_2_frame_ = std::make_shared<rs2::video_frame>(local_frame);
      }
      // RCLCPP_INFO(this->get_logger(), "Fisheye: %d", fisheye_index);
      // runFisheye(*fisheye_frame_, fisheye_index);
    }
  }


  if (imu_available_) {
    runImu(*accel_frame_, *gyro_frame_);
  }

  // if (pose_available_) {
  //   runPose(*pose_frame_);
  // }

  // if (color_available_) {
  //   runColor(*color_frame_);
  // }

  return;
}

void RealsenseInterface::runPose(const rs2::pose_frame & pose_frame_)
{
  rs2_pose pose_data = pose_frame_.as<rs2::pose_frame>().get_pose_data();
  // Realsense timestamp is in microseconds
  rclcpp::Time timestamp = this->now();

  realsense_pose_ = tf2::Transform(
    tf2::Quaternion(
      pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z,
      pose_data.rotation.w),
    tf2::Vector3(pose_data.translation.x, pose_data.translation.y, pose_data.translation.z));

  // This should be agnostic to the device pitch and roll (but not yaw) initial orientation
  tf2::Transform base_link_pose =
    base_link_to_realsense_pose_odom_ * realsense_link_to_realsense_pose_ * realsense_pose_ *
    realsense_link_to_realsense_pose_.inverse() * base_link_to_realsense_link_.inverse();

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_link_frame_;
  odom_msg.header.stamp = timestamp;

  odom_msg.pose.pose.position.x = base_link_pose.getOrigin().getX();
  odom_msg.pose.pose.position.y = base_link_pose.getOrigin().getY();
  odom_msg.pose.pose.position.z = base_link_pose.getOrigin().getZ();
  odom_msg.pose.pose.orientation = tf2::toMsg(
    tf2::Quaternion(
      base_link_pose.getRotation().getX(), base_link_pose.getRotation().getY(),
      base_link_pose.getRotation().getZ(), base_link_pose.getRotation().getW()));

  // Check if odom orientation is valid
  if (std::isnan(odom_msg.pose.pose.orientation.x) ||
    std::isnan(odom_msg.pose.pose.orientation.y) ||
    std::isnan(odom_msg.pose.pose.orientation.z) ||
    std::isnan(odom_msg.pose.pose.orientation.w))
  {
    RCLCPP_ERROR(get_logger(), "Odom orientation is NaN");
    return;
  }
  // Check if odom orientation module is 1
  if (std::abs(
      odom_msg.pose.pose.orientation.x * odom_msg.pose.pose.orientation.x +
      odom_msg.pose.pose.orientation.y * odom_msg.pose.pose.orientation.y +
      odom_msg.pose.pose.orientation.z * odom_msg.pose.pose.orientation.z +
      odom_msg.pose.pose.orientation.w * odom_msg.pose.pose.orientation.w - 1.0) > 0.01)
  {
    RCLCPP_ERROR(get_logger(), "Odom orientation module is not 1");
    return;
  }

  // New method to obtain linear velocity
  tf2::Vector3 rs_linear_velocity(pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z);
  tf2::Vector3 rs_angular_velocity(pose_data.angular_velocity.x, pose_data.angular_velocity.y,
    pose_data.angular_velocity.z);

  // Drone_velocity = RS_velocity - RS_ang_vel x RS_position
  tf2::Vector3 linear_velocity =
    rs_linear_velocity -
    tf2::Vector3(
    rs_angular_velocity.getX(), rs_angular_velocity.getY(),
    rs_angular_velocity.getZ())
    .cross(
    tf2::Vector3(
      base_link_to_realsense_link_.getOrigin().getX(),
      base_link_to_realsense_link_.getOrigin().getY(),
      base_link_to_realsense_link_.getOrigin().getZ()));

  tf2::Vector3 odom_linear_velocity = base_link_to_realsense_pose_odom_.getBasis() *
    realsense_link_to_realsense_pose_.getBasis() *
    linear_velocity;

  tf2::Vector3 base_link_linear_velocity =
    base_link_pose.inverse().getBasis() * odom_linear_velocity;

  odom_msg.twist.twist.linear.x = base_link_linear_velocity.getX();
  odom_msg.twist.twist.linear.y = base_link_linear_velocity.getY();
  odom_msg.twist.twist.linear.z = base_link_linear_velocity.getZ();

  tf2::Vector3 odom_angular_velocity = base_link_to_realsense_pose_odom_.getBasis() *
    realsense_link_to_realsense_pose_.getBasis() *
    rs_angular_velocity;

  tf2::Vector3 base_link_angular_velocity =
    base_link_pose.inverse().getBasis() * odom_angular_velocity;

  odom_msg.twist.twist.angular.x = base_link_angular_velocity.getX();
  odom_msg.twist.twist.angular.y = base_link_angular_velocity.getY();
  odom_msg.twist.twist.angular.z = base_link_angular_velocity.getZ();


  pose_sensor_->updateData(odom_msg);

  return;
}

void RealsenseInterface::runImu(
  const rs2::motion_frame & accel_frame_,
  const rs2::motion_frame & gyro_frame_)
{
  rs2_vector accel_data = accel_frame_.get_motion_data();
  rs2_vector gyro_data = gyro_frame_.get_motion_data();

  rclcpp::Time timestamp;
  // get time from frame metadata
  // Realsense timestamp is in microseconds
  double accel_time = accel_frame_.get_timestamp() * 1000;
  double gyro_time = gyro_frame_.get_timestamp() * 1000;
  if (gyro_time > accel_time) {
    timestamp = rclcpp::Time(gyro_time);
  } else {
    timestamp = rclcpp::Time(accel_time);
  }

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = imu_sensor_frame_;
  imu_msg.header.stamp = timestamp;

  imu_msg.linear_acceleration.x = accel_data.x;
  imu_msg.linear_acceleration.y = accel_data.y;
  imu_msg.linear_acceleration.z = accel_data.z;

  imu_msg.angular_velocity.x = gyro_data.x;
  imu_msg.angular_velocity.y = gyro_data.y;
  imu_msg.angular_velocity.z = gyro_data.z;

  imu_sensor_->updateData(imu_msg);

  return;
}

void RealsenseInterface::runColor(const rs2::video_frame & _color_frame)
{
  // Realsense timestamp is in microseconds
  rclcpp::Time timestamp = rclcpp::Time(_color_frame.get_timestamp() * 1000);
  sensor_msgs::msg::Image color_msg;
  color_msg.header.frame_id = color_sensor_frame_;
  color_msg.header.stamp = timestamp;
  color_msg.height = _color_frame.get_height();
  color_msg.width = _color_frame.get_width();
  color_msg.encoding = sensor_msgs::image_encodings::RGB8;
  color_msg.is_bigendian = false;
  color_msg.step = color_msg.width * 3;
  color_msg.data.resize(color_msg.height * color_msg.step);
  memcpy(color_msg.data.data(), _color_frame.get_data(), color_msg.data.size());

  _color_frame.get_data();
  color_sensor_->updateData(color_msg);

  return;
}

void RealsenseInterface::runFisheye(const rs2::video_frame & _frame, const int index)
{
  // TODO(dps): remove OpenCV dependency from as2_core
  // Realsense timestamp is in microseconds, convert to milliseconds
  // rclcpp::Time timestamp = rclcpp::Time(_frame.get_timestamp() * 1000);
  rclcpp::Time timestamp = this->get_clock()->now();

  // Prepare sensor_msgs::msg::Image message
  sensor_msgs::msg::Image image_msg;
  image_msg.header.stamp = timestamp;
  image_msg.height = _frame.get_height();
  image_msg.width = _frame.get_width();
  image_msg.encoding = sensor_msgs::image_encodings::MONO8;
  image_msg.is_bigendian = false;
  image_msg.step = image_msg.width;

  // Convert the data to an OpenCV Mat
  cv::Mat original_image(image_msg.height, image_msg.width, CV_8UC1,
    const_cast<void *>(_frame.get_data()));

  cv_bridge::CvImage cv_image;
  cv_image.header = image_msg.header;  // Set the header (timestamp, frame_id)
  cv_image.encoding = sensor_msgs::image_encodings::MONO8;  // Grayscale image

  if (undistort_image_) {
    // Get the new camera matrix for undistortion
    static bool generate_rect_matrix = true;
    if (generate_rect_matrix) {
      static bool generate_cam1 = true;
      static bool generate_cam2 = true;

      if (index == 1 && generate_cam1) {
        fisheye1_sensor_->generateUndistortionParameters(original_image.size());
        generate_cam1 = false;
      }
      if (index == 2 && generate_cam2) {
        fisheye2_sensor_->generateUndistortionParameters(original_image.size());
        generate_cam2 = false;
      }
      if ((!generate_cam1) && (!generate_cam2)) {
        generate_rect_matrix = false;
      }
    }

    cv::Mat undistorted_matrix;
    // Initialize the CameraInfo object
    sensor_msgs::msg::CameraInfo camera_info;
    if (index == 1) {
      // Fetch CameraInfo for fisheye 1
      camera_info = fisheye1_sensor_->getUndistortedCameraInfo();
      undistorted_matrix =
        cv::Mat(3, 3, CV_64F, reinterpret_cast<void *>(fisheye1_sensor_->getCameraInfo().k.data()));
    }
    if (index == 2) {
      // Fetch CameraInfo for fisheye 2
      camera_info = fisheye2_sensor_->getUndistortedCameraInfo();
      undistorted_matrix =
        cv::Mat(3, 3, CV_64F, reinterpret_cast<void *>(fisheye2_sensor_->getCameraInfo().k.data()));
    }
    // Convert the CameraInfo's K (intrinsic matrix) to cv::Mat
    cv::Mat K = cv::Mat(3, 3, CV_64F, reinterpret_cast<void *>(camera_info.k.data()));
    // Convert the CameraInfo's D (distortion coefficients) to cv::Mat
    cv::Mat D = cv::Mat(4, 1, CV_64F, reinterpret_cast<void *>(camera_info.d.data()));

    // Undistort the image using fisheye model
    cv::Mat undistorted_image;
    cv::fisheye::undistortImage(original_image, undistorted_image, K, D, undistorted_matrix);
    cv_image.image = undistorted_image;
  } else {
    cv_image.image = original_image;
  }

  // Convert the image to a ROS message using cv_bridge
  try {
    image_msg = *cv_image.toImageMsg();
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Update the frame_id and publish based on index
  if (index == 1) {
    image_msg.header.frame_id = fisheye1_sensor_frame_;
    fisheye1_sensor_->updateData(image_msg);
  }
  if (index == 2) {
    image_msg.header.frame_id = fisheye2_sensor_frame_;
    fisheye2_sensor_->updateData(image_msg);
  }

  return;
}


void RealsenseInterface::setupPoseTransforms(
  const std::array<double, 3> & device_t,
  const std::array<double, 3> & device_r)
{
  // Create a quaternion from the Euler angles of the device static position
  tf2::Quaternion device_q;
  device_q.setRPY(device_r[0], device_r[1], device_r[2]);

  // Change from rs_link FLU to rs OWN.
  const float rs_link2pose_rot[3] = {M_PI_2, 0, -M_PI_2};
  tf2::Quaternion rs_link2pose_q;
  tf2::Quaternion base_link2pose_odom_q;
  rs_link2pose_q.setRPY(rs_link2pose_rot[0], rs_link2pose_rot[1], rs_link2pose_rot[2]);
  base_link2pose_odom_q.setRPY(0.0, 0.0, device_r[2]);

  // Base link to Realsense link FRU
  base_link_to_realsense_link_ =
    tf2::Transform(device_q, tf2::Vector3(device_t[0], device_t[1], device_t[2]));

  // Realsense link to Realsense pose
  realsense_link_to_realsense_pose_ = tf2::Transform(rs_link2pose_q, tf2::Vector3(0, 0.0091, 0));

  // Base link to Realsense pose odom in FRU
  // This should be agnostic to the device pitch and roll (but not yaw) initial orientation
  base_link_to_realsense_pose_odom_ =
    tf2::Transform(base_link2pose_odom_q, tf2::Vector3(device_t[0], device_t[1], device_t[2]));

  // Realsense pose
  realsense_pose_ = tf2::Transform::getIdentity();
}

void RealsenseInterface::setupCamera(
  const std::shared_ptr<as2::sensors::Camera> & _camera,
  const rs2_stream _rs2_stream,
  const int _index,
  const std::string & _encoding,
  const std::string & _sensor_frame)
{
  sensor_msgs::msg::CameraInfo camera_info;
  // rs2::video_stream_profile profile =
  //  profile.get_stream(RS2_STREAM_FISHEYE, _index).as<rs2::video_stream_profile>();
  camera_info.header.frame_id = _sensor_frame;
  // rs2_intrinsics intrinsics = profile.get_intrinsics();
  // auto intrinsics = pipe_.get_active_profile()
  //   .get_stream(_rs2_stream, _index)
  //   .as<rs2::video_stream_profile>()
  //   .get_intrinsics();
  auto profile = pipe_.get_active_profile();
  auto camera_stream = profile.get_stream(_rs2_stream, _index).as<rs2::video_stream_profile>();
  auto intrinsics = camera_stream.get_intrinsics();

  camera_info.width = intrinsics.width;
  camera_info.height = intrinsics.height;
  camera_info.k[0] = intrinsics.fx;
  camera_info.k[2] = intrinsics.ppx;
  camera_info.k[4] = intrinsics.fy;
  camera_info.k[5] = intrinsics.ppy;
  camera_info.k[8] = 1;

  // Distortion model
  camera_info.distortion_model = rs2_distortion_to_string(intrinsics.model);
  // Distorsion coefficients
  camera_info.d.reserve(5);
  for (int i = 0; i < 5; i++) {
    camera_info.d.emplace_back(intrinsics.coeffs[i]);
  }

  // rectification matrix
  camera_info.r[0] = 1;
  camera_info.r[4] = 1;
  camera_info.r[8] = 1;
  // projection matrix
  camera_info.p[0] = camera_info.k[0];
  camera_info.p[2] = camera_info.k[2];
  camera_info.p[5] = camera_info.k[4];
  camera_info.p[6] = camera_info.k[5];
  camera_info.p[10] = 1;

  _camera->setCameraInfo(camera_info);

  auto pose_stream = profile.get_stream(RS2_STREAM_POSE).as<rs2::pose_stream_profile>();
  rs2_extrinsics extrinsics = camera_stream.get_extrinsics_to(pose_stream);

  // Create rotation matrix from extrinsics rotation array (3x3 matrix)
  tf2::Matrix3x3 rotation_matrix(
    extrinsics.rotation[0], extrinsics.rotation[1], extrinsics.rotation[2],
    extrinsics.rotation[3], extrinsics.rotation[4], extrinsics.rotation[5],
    extrinsics.rotation[6], extrinsics.rotation[7], extrinsics.rotation[8]
  );
  // Convert rotation matrix to quaternion
  tf2::Quaternion quaternion;
  rotation_matrix.getRotation(quaternion);

  _camera->setStaticTransform(
    _sensor_frame,
    pose_link_frame_,
    extrinsics.translation[0],
    extrinsics.translation[1],
    extrinsics.translation[2],
    quaternion.x(),
    quaternion.y(),
    quaternion.z(),
    quaternion.w());
}

bool RealsenseInterface::identifyDevice()
{
  rs2::context ctx;
  auto devices = ctx.query_devices();
  if (devices.size() == 0) {
    return false;
  }
  for (auto dev : devices) {
    if (dev.supports(RS2_CAMERA_INFO_NAME)) {
      auto device_name = std::string(dev.get_info(RS2_CAMERA_INFO_NAME));
      RCLCPP_INFO(get_logger(), "Found device: %s", device_name.c_str());

      if (verbose_) {
        if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
          RCLCPP_INFO(
            get_logger(), "Device serial number: %s",
            dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        }
        if (dev.supports(RS2_CAMERA_INFO_PRODUCT_ID)) {
          RCLCPP_INFO(
            get_logger(), "Device product ID: %s",
            dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID));
        }
        if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION)) {
          RCLCPP_INFO(
            get_logger(), "Device firmware version: %s",
            dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION));
        }
        if (dev.supports(RS2_CAMERA_INFO_PHYSICAL_PORT)) {
          RCLCPP_INFO(
            get_logger(), "Device physical port: %s",
            dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT));
        }
        if (dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
          RCLCPP_INFO(
            get_logger(), "Device USB type descriptor: %s",
            dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));
        }
        if (dev.supports(RS2_CAMERA_INFO_DEBUG_OP_CODE)) {
          RCLCPP_INFO(
            get_logger(), "Device supports debug op code: %s",
            dev.get_info(RS2_CAMERA_INFO_DEBUG_OP_CODE));
        }
        if (dev.supports(RS2_CAMERA_INFO_CAMERA_LOCKED)) {
          RCLCPP_INFO(
            get_logger(), "Device supports camera locked: %s",
            dev.get_info(RS2_CAMERA_INFO_CAMERA_LOCKED));
        }
      }

      identifySensors(dev);
    }
  }

  return true;
}

bool RealsenseInterface::identifySensors(const rs2::device & dev)
{
  std::vector<rs2::sensor> dev_sensors = dev.query_sensors();

  bool gyro_available = false;
  bool accel_available = false;

  RCLCPP_INFO(get_logger(), "Device has following sensors:");

  for (auto sensor : dev_sensors) {
    RCLCPP_INFO(get_logger(), " - %s", sensor.get_info(RS2_CAMERA_INFO_NAME));

    for (auto profile : sensor.get_stream_profiles()) {
      if (profile.stream_type() == RS2_STREAM_GYRO) {
        gyro_available = true;
      }
      if (profile.stream_type() == RS2_STREAM_ACCEL) {
        accel_available = true;
      }
      if (profile.stream_type() == RS2_STREAM_DEPTH) {
        depth_available_ = true;
      }
      if (profile.stream_type() == RS2_STREAM_COLOR) {
        color_available_ = true;
      }
      if (profile.stream_type() == RS2_STREAM_POSE) {
        pose_available_ = true;
      }
      if (profile.stream_type() == RS2_STREAM_FISHEYE) {
        fisheye_available_ = true;
      }
    }
  }

  RCLCPP_INFO(get_logger(), "Device has following streams:");
  if (gyro_available && accel_available) {
    imu_available_ = true;
    RCLCPP_INFO(get_logger(), " - IMU stream");
  }

  if (depth_available_) {
    RCLCPP_INFO(get_logger(), " - Depth stream");
  }

  if (color_available_) {
    RCLCPP_INFO(get_logger(), " - Color stream");
  }

  if (pose_available_) {
    RCLCPP_INFO(get_logger(), " - Pose stream");
  }

  if (fisheye_available_) {
    RCLCPP_INFO(get_logger(), " - Fisheye stream");
  }

  return true;
}

void RealsenseInterface::setStaticTransform(
  const std::string & _link_frame,
  const std::string & _ref_frame,
  const std::array<double, 3> & _translation,
  const std::array<double, 3> & _rotation)
{
  // Publish static transform
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = _ref_frame;
  transform_stamped.child_frame_id = _link_frame;
  transform_stamped.transform.translation.x = _translation[0];
  transform_stamped.transform.translation.y = _translation[1];
  transform_stamped.transform.translation.z = _translation[2];
  tf2::Quaternion q;
  q.setRPY(_rotation[0], _rotation[1], _rotation[2]);
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();
  tf_static_broadcaster_->sendTransform(transform_stamped);
}

}  // namespace real_sense_interface
