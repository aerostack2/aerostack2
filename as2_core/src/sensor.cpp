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
 *  \file       sensor.cpp
 *  \brief      Sensor class for AS2 implementation file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#include "as2_core/sensor.hpp"

namespace as2
{
namespace sensors
{

// TFStatic

TFStatic::TFStatic(rclcpp::Node * node_ptr)
: node_ptr_(node_ptr) {}

TFStatic::~TFStatic() {}

void TFStatic::setStaticTransform(
  const std::string & frame_id, const std::string & parent_frame_id, float x, float y, float z,
  float roll, float pitch, float yaw)
{
  // convert to quaternion and set the transform
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  setStaticTransform(frame_id, parent_frame_id, x, y, z, q.x(), q.y(), q.z(), q.w());
}

void TFStatic::setStaticTransform(
  const std::string & frame_id, const std::string & parent_frame_id, float x, float y, float z,
  float qx, float qy, float qz, float qw)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = node_ptr_->now();
  transformStamped.header.frame_id = parent_frame_id;
  transformStamped.child_frame_id = frame_id;
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = z;
  transformStamped.transform.rotation.x = qx;
  transformStamped.transform.rotation.y = qy;
  transformStamped.transform.rotation.z = qz;
  transformStamped.transform.rotation.w = qw;
  setStaticTransform_(transformStamped);
}

void TFStatic::setStaticTransform(
  const geometry_msgs::msg::TransformStamped & transformStamped)
{
  setStaticTransform_(transformStamped);
}

void TFStatic::setStaticTransform_(
  const geometry_msgs::msg::TransformStamped & transformStamped)
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster(node_ptr_);
  static_broadcaster.sendTransform(transformStamped);
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Static transform published: %s -> %s",
    transformStamped.header.frame_id.c_str(),
    transformStamped.child_frame_id.c_str());
}

const rclcpp::Node * TFStatic::getNode() const
{
  return node_ptr_;
}

// TFDynamic

TFDynamic::TFDynamic(rclcpp::Node * node_ptr)
: node_ptr_(node_ptr)
{
  dynamic_tf_broadcaster_ptr_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_ptr);
}

TFDynamic::~TFDynamic()
{
  dynamic_tf_broadcaster_ptr_.reset();
}

void TFDynamic::setDynamicTransform(const geometry_msgs::msg::TransformStamped & transform)
{
  dynamic_tf_broadcaster_ptr_->sendTransform(transform);
}

void TFDynamic::setDynamicTransform(
  const std::string & frame_id, const std::string & parent_frame_id, float x, float y, float z,
  float qx, float qy, float qz, float qw)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_ptr_->now();
  transform.header.frame_id = parent_frame_id;
  transform.child_frame_id = frame_id;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  transform.transform.rotation.x = qx;
  transform.transform.rotation.y = qy;
  transform.transform.rotation.z = qz;
  transform.transform.rotation.w = qw;
  setDynamicTransform(transform);
}

void TFDynamic::setDynamicTransform(
  const std::string & frame_id, const std::string & parent_frame_id, float x, float y, float z,
  float roll, float pitch, float yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  setDynamicTransform(frame_id, parent_frame_id, x, y, z, q.x(), q.y(), q.z(), q.w());
}

std::shared_ptr<tf2_ros::TransformBroadcaster> TFDynamic::getTransformBroadcaster() const
{
  return dynamic_tf_broadcaster_ptr_;
}

const rclcpp::Node * TFDynamic::getNode() const
{
  return node_ptr_;
}

// GenericSensor

GenericSensor::GenericSensor(as2::Node * node_ptr, const float pub_freq)
:  pub_freq_(pub_freq)
{
  if (pub_freq > 0.0f) {
    timer_ = node_ptr->create_timer(
      std::chrono::duration<double>(1.0 / pub_freq),
      std::bind(&GenericSensor::timerCallback, this));
  }
}

GenericSensor::~GenericSensor()
{
  timer_.reset();
}

void GenericSensor::dataUpdated()
{
  if (pub_freq_ <= 0.0f) {
    publishData();
  }
  // else: Timer will publish the data
}

void GenericSensor::timerCallback()
{
  publishData();
}

// Camera

Camera::Camera(
  as2::Node * node_ptr,
  const std::string & prefix,
  const float pub_freq,
  bool add_sensor_measurements_base,
  const std::string & camera_link)
: TFStatic(node_ptr), GenericSensor(node_ptr, pub_freq), node_ptr_(node_ptr)
{
  // Check if ROS 2 camera_nameparameter is set
  std::string ros_param_prefix = processParametersPrefix(prefix);

  // Get camera name from ROS 2 parameters
  try {
    node_ptr->declare_parameter<std::string>(ros_param_prefix + "camera_name", "");
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
    RCLCPP_ERROR(
      node_ptr->get_logger(),
      "Camera sensor constructor failed."
      "Parameter %s already declared."
      "Cannot create two camera sensors with the same name."
      "%s", (ros_param_prefix + "camera_name").c_str(), e.what());
  }
  node_ptr->get_parameter(ros_param_prefix + "camera_name", camera_name_);

  bool enable_parameters = true;
  if (camera_name_.empty()) {
    // If camera name is empty and no ROS 2 parameter is set, can not create sensor
    if (prefix.empty()) {
      RCLCPP_ERROR(
        node_ptr->get_logger(),
        "Camera sensor constructor failed. "
        "Prefix is empty and ROS 2 parameter %s is not set.",
        (ros_param_prefix + "camera_name").c_str());
      throw std::runtime_error(
              "Camera sensor constructor failed. "
              "Prefix is empty and no ROS 2 parameter is set.");
      return;
      // Do not use ROS 2 parameters
    } else {
      camera_name_ = prefix;
      RCLCPP_DEBUG(
        node_ptr->get_logger(),
        "Camera name parameter '%s' not set.\n"
        "You should call setCameraInfo and setCameraLinkTransform methods by yourself.",
        (ros_param_prefix + "camera_name").c_str());
      enable_parameters = false;
    }
  }

  camera_base_topic_ = SensorData<sensor_msgs::msg::CameraInfo>::processTopicName(
    camera_name_ + "/image", add_sensor_measurements_base);

  camera_link_frame_ = as2::tf::generateTfName(
    node_ptr_->get_namespace(), camera_name_ + '/' + camera_link);

  if (enable_parameters) {
    // Read camera info from ROS parameters
    readCameraInfoFromROSParameters(prefix);

    // Read camera transform from ROS parameters
    readCameraTranformFromROSParameters(prefix);
  }
}

Camera::~Camera()
{
  it_camera_publisher_ptr_.reset();
}

void Camera::updateData(const sensor_msgs::msg::Image & img)
{
  if (!setup_) {
    setup();
  }

  image_data_ = img;
  // Update timestamp
  camera_info_.header.stamp = img.header.stamp;
  // Add frame_id
  image_data_.header.frame_id = camera_link_frame_;
  camera_info_.header.frame_id = camera_link_frame_;
  dataUpdated();
}

void Camera::updateData(const cv::Mat & img)
{
  sensor_msgs::msg::Image img_msg;
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = node_ptr_->now();
  cv_image.header.frame_id = camera_link_frame_;
  cv_image.encoding = encoding_;
  cv_image.image = img;
  cv_image.toImageMsg(img_msg);

  updateData(img_msg);
}

void Camera::setCameraInfo(
  const sensor_msgs::msg::CameraInfo & camera_info)
{
  camera_info_ = camera_info;
}

void Camera::setEncoding(const std::string & encoding)
{
  encoding_ = encoding;
}

void Camera::setCameraLinkTransform(
  const std::string & parent_frame_id,
  const float x, const float y, const float z, const float roll, const float pitch,
  const float yaw)
{
  this->setStaticTransform(camera_link_frame_, parent_frame_id, x, y, z, roll, pitch, yaw);
}

void Camera::readCameraInfoFromROSParameters(
  const std::string & prefix)
{
  std::string ros_param_prefix = processParametersPrefix(prefix);

  node_ptr_->declare_parameter<std::string>(ros_param_prefix + "encoding");
  node_ptr_->declare_parameter<int>(ros_param_prefix + "image_width");
  node_ptr_->declare_parameter<int>(ros_param_prefix + "image_height");
  node_ptr_->declare_parameter<std::vector<double>>(ros_param_prefix + "camera_matrix.data");
  node_ptr_->declare_parameter<std::string>(ros_param_prefix + "distortion_model");
  node_ptr_->declare_parameter<std::vector<double>>(
    ros_param_prefix + "distortion_coefficients.data");
  node_ptr_->declare_parameter<std::vector<double>>(ros_param_prefix + "rectification_matrix.data");
  node_ptr_->declare_parameter<std::vector<double>>(ros_param_prefix + "projection_matrix.data");

  // Get parameters
  sensor_msgs::msg::CameraInfo cam_info;
  std::vector<double> matrix;
  std::string encoding;
  node_ptr_->get_parameter(ros_param_prefix + "encoding", encoding);
  setEncoding(encoding);
  node_ptr_->get_parameter(ros_param_prefix + "image_width", cam_info.width);
  node_ptr_->get_parameter(ros_param_prefix + "image_height", cam_info.height);
  node_ptr_->get_parameter(ros_param_prefix + "camera_matrix.data", matrix);
  convertVectorToArray(matrix, cam_info.k);
  node_ptr_->get_parameter(ros_param_prefix + "distortion_model", cam_info.distortion_model);
  node_ptr_->get_parameter(ros_param_prefix + "distortion_coefficients.data", cam_info.d);
  node_ptr_->get_parameter(ros_param_prefix + "rectification_matrix.data", matrix);
  convertVectorToArray(matrix, cam_info.r);
  node_ptr_->get_parameter(ros_param_prefix + "projection_matrix.data", matrix);
  convertVectorToArray(matrix, cam_info.p);

  // Set header
  cam_info.header.frame_id = camera_link_frame_;

  // Set camera info
  setCameraInfo(cam_info);
}

void Camera::readCameraTranformFromROSParameters(
  const std::string & prefix)
{
  std::string ros_param_prefix = processParametersPrefix(prefix);

  // Declare camera parameters
  node_ptr_->declare_parameter<std::string>(ros_param_prefix + "camera_transform.parent_frame");
  node_ptr_->declare_parameter<double>(ros_param_prefix + "camera_transform.x");
  node_ptr_->declare_parameter<double>(ros_param_prefix + "camera_transform.y");
  node_ptr_->declare_parameter<double>(ros_param_prefix + "camera_transform.z");
  node_ptr_->declare_parameter<double>(ros_param_prefix + "camera_transform.roll");
  node_ptr_->declare_parameter<double>(ros_param_prefix + "camera_transform.pitch");
  node_ptr_->declare_parameter<double>(ros_param_prefix + "camera_transform.yaw");

  // Get parameters
  std::string parent_frame;
  double x, y, z, roll, pitch, yaw;
  node_ptr_->get_parameter(ros_param_prefix + "camera_transform.parent_frame", parent_frame);
  node_ptr_->get_parameter(ros_param_prefix + "camera_transform.x", x);
  node_ptr_->get_parameter(ros_param_prefix + "camera_transform.y", y);
  node_ptr_->get_parameter(ros_param_prefix + "camera_transform.z", z);
  node_ptr_->get_parameter(ros_param_prefix + "camera_transform.roll", roll);
  node_ptr_->get_parameter(ros_param_prefix + "camera_transform.pitch", pitch);
  node_ptr_->get_parameter(ros_param_prefix + "camera_transform.yaw", yaw);
  parent_frame = as2::tf::generateTfName(node_ptr_->get_namespace(), parent_frame);
  setCameraLinkTransform(parent_frame, x, y, z, roll, pitch, yaw);
}

void Camera::setup()
{
  it_camera_publisher_ptr_ = std::make_shared<image_transport::CameraPublisher>(
    node_ptr_, camera_base_topic_);
  setup_ = true;
}

void Camera::publishData()
{
  it_camera_publisher_ptr_->publish(image_data_, camera_info_);
}

std::string Camera::processParametersPrefix(
  const std::string & prefix)
{
  std::string _prefix;
  // If not empty and not ending with a dot, add a dot
  if (!prefix.empty() && prefix.back() != '.') {
    _prefix = prefix + ".";
  }
  return _prefix;
}


// Grount Truth Sensor

GroundTruth::GroundTruth(
  as2::Node * node_ptr, const float pub_freq,
  const std::string & topic_name_base)
: GenericSensor(node_ptr, pub_freq)
{
  const std::string pose_topic_name = topic_name_base + as2_names::topics::ground_truth::pose;
  const std::string twist_topic_name = topic_name_base + as2_names::topics::ground_truth::twist;

  pose_sensor_ = std::make_shared<SensorData<geometry_msgs::msg::PoseStamped>>(
    pose_topic_name, node_ptr, false);
  twist_sensor_ = std::make_shared<SensorData<geometry_msgs::msg::TwistStamped>>(
    twist_topic_name, node_ptr, false);
}

GroundTruth::~GroundTruth()
{
  // Clean up ROS 2 publishers
  pose_sensor_.reset();
  twist_sensor_.reset();
}

void GroundTruth::updateData(const geometry_msgs::msg::PoseStamped & pose_msg)
{
  pose_sensor_->setData(pose_msg);
  dataUpdated();
}

void GroundTruth::updateData(const geometry_msgs::msg::TwistStamped & twist_msg)
{
  twist_sensor_->setData(twist_msg);
  dataUpdated();
}

void GroundTruth::updateData(
  const geometry_msgs::msg::PoseStamped & pose_msg,
  const geometry_msgs::msg::TwistStamped & twist_msg)
{
  pose_sensor_->setData(pose_msg);
  twist_sensor_->setData(twist_msg);
  dataUpdated();
}

void GroundTruth::publishData()
{
  pose_sensor_->publish();
  twist_sensor_->publish();
}

// Gimbal

Gimbal::Gimbal(
  const std::string & gimbal_id,
  const std::string & gimbal_base_id,
  as2::Node * node_ptr,
  const float pub_freq,
  bool add_sensor_measurements_base)
: TFStatic(node_ptr), TFDynamic(node_ptr), GenericSensor(node_ptr, pub_freq),
  SensorData<geometry_msgs::msg::PoseStamped>(gimbal_id, node_ptr, add_sensor_measurements_base)
{
  gimbal_frame_id_ = as2::tf::generateTfName(node_ptr->get_namespace(), gimbal_id);
  gimbal_base_frame_id_ = as2::tf::generateTfName(node_ptr->get_namespace(), gimbal_base_id);
}

Gimbal::~Gimbal() {}

void Gimbal::setGimbalBaseTransform(
  const geometry_msgs::msg::Transform & gimbal_base_transform,
  const std::string & gimbal_parent_frame_id)
{
  // Set static transform between gimbal_base and gimbal_parent_frame_id
  std::string _gimbal_parent_frame_id = as2::tf::generateTfName(
    TFStatic::getNode()->get_namespace(), gimbal_parent_frame_id);
  geometry_msgs::msg::TransformStamped gimbal_base_transform_stamped;
  gimbal_base_transform_stamped.header.stamp = TFStatic::getNode()->now();
  gimbal_base_transform_stamped.header.frame_id = _gimbal_parent_frame_id;
  gimbal_base_transform_stamped.child_frame_id = gimbal_base_frame_id_;
  gimbal_base_transform_stamped.transform = gimbal_base_transform;
  setStaticTransform(gimbal_base_transform_stamped);
}

void Gimbal::updateData(const geometry_msgs::msg::PoseStamped & pose_msg)
{
  SensorData<geometry_msgs::msg::PoseStamped>::setData(pose_msg);
  gimbal_transform_.header.stamp = pose_msg.header.stamp;
  gimbal_transform_.header.frame_id = gimbal_base_frame_id_;
  gimbal_transform_.child_frame_id = gimbal_frame_id_;
  gimbal_transform_.transform.translation.x = pose_msg.pose.position.x;
  gimbal_transform_.transform.translation.y = pose_msg.pose.position.y;
  gimbal_transform_.transform.translation.z = pose_msg.pose.position.z;
  gimbal_transform_.transform.rotation = pose_msg.pose.orientation;
  GenericSensor::dataUpdated();
}

void Gimbal::updateData(const geometry_msgs::msg::QuaternionStamped & orientation_msg)
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = orientation_msg.header;
  pose_msg.pose.position.x = gimbal_transform_.transform.translation.x;
  pose_msg.pose.position.y = gimbal_transform_.transform.translation.y;
  pose_msg.pose.position.z = gimbal_transform_.transform.translation.z;
  pose_msg.pose.orientation = orientation_msg.quaternion;
  updateData(pose_msg);
}

const std::string & Gimbal::getGimbalFrameId() const
{
  return gimbal_frame_id_;
}

const std::string & Gimbal::getGimbalBaseFrameId() const
{
  return gimbal_base_frame_id_;
}

void Gimbal::publishData()
{
  SensorData<geometry_msgs::msg::PoseStamped>::publish();
  setDynamicTransform(gimbal_transform_);
}


}  // namespace sensors
}  // namespace as2
