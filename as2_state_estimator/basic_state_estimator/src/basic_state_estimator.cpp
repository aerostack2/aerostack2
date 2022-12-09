/*!*******************************************************************************************
 *  \file       basic_state_estimator.cpp
 *  \brief      A basic state estimator for AeroStack2
 *  \authors    Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *              Pedro Arias Pérez
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

#include "basic_state_estimator.hpp"
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>

BasicStateEstimator::BasicStateEstimator() : as2::Node("basic_state_estimator") {
  this->declare_parameter<bool>("odom_only", false);
  this->declare_parameter<bool>("ground_truth", false);
  this->declare_parameter<bool>("sensor_fusion", false);
  this->declare_parameter<bool>("rectified_localization", false);
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("global_ref_frame", "global_ref_frame");
}

void BasicStateEstimator::run() {
  if (!start_run_) {
    return;
  }
  // TODO: SENSOR FUSION
  geometry_msgs::msg::TransformStamped map2odom_tf;
  map2odom_tf = calculateLocalization();
  updateOdomTfDrift(odom2baselink_tf_, map2odom_tf);
  // if (rectified_localization_) {
  //   // TODO: Search the tf with the same frames
  //   try {
  //     // std::string rectified_frame = baselink_frame_; frame_rectified_tf_.child_frame_id
  //     std::string ref_frame = "earth";
  //     auto pose_transform   = tf_buffer_->lookupTransform(
  //         ref_frame, frame_rectified_tf_.child_frame_id, tf2::TimePointZero);
  //     if (pose_transform.header.frame_id == ref_frame &&
  //         pose_transform.child_frame_id == frame_rectified_tf_.child_frame_id) {
  //       ref2ref_rectified_tf_.header.frame_id = frame_rectified_tf_.header.frame_id;
  //       ref2ref_rectified_tf_.child_frame_id  = ref_frame;
  //       updateRefTfRectification(frame_rectified_tf_.transform, pose_transform.transform);
  //     } else {
  //       RCLCPP_WARN(get_logger(), "Tranformation nod found: %s - %s", ref_frame.c_str(),
  //                   frame_rectified_tf_.child_frame_id.c_str());
  //     }
  //   } catch (tf2::TransformException &ex) {
  //     RCLCPP_WARN(this->get_logger(), "Transform Failure: %s\n",
  //                 ex.what());  // Print exception which was caught
  //   }
  // }
  publishTfs();
  getGlobalRefState();
  publishStateEstimation();
}

void BasicStateEstimator::setupNode() {
  // Initialize the transform broadcaster
  tf_broadcaster_       = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tfstatic_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_buffer_            = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_          = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string base_frame;
  std::string global_ref_frame;
  this->get_parameter("base_frame", base_frame);
  this->get_parameter("global_ref_frame", global_ref_frame);
  this->get_parameter("odom_only", odom_only_);
  this->get_parameter("ground_truth", ground_truth_);
  this->get_parameter("sensor_fusion", sensor_fusion_);
  this->get_parameter("rectified_localization", rectified_localization_);

  if (odom_only_) {
    RCLCPP_INFO(get_logger(), "ODOM ONLY MODE");
  }

  if (ground_truth_) {
    RCLCPP_INFO(get_logger(), "GROUND TRUTH MODE");
  }

  if (sensor_fusion_) {
    RCLCPP_INFO(get_logger(), "SENSOR FUSION MODE");
  }

  if (!odom_only_ & !ground_truth_ & !sensor_fusion_) {
    RCLCPP_ERROR(get_logger(), "NO ESTIMATION MODE ENABLED");
    RCLCPP_ERROR(get_logger(), "DEFAULT: ODOM ONLY ACTIVATED");
    odom_only_ = true;
  }

  if (rectified_localization_) {
    RCLCPP_INFO(get_logger(), "RECTIFIED LOCALIZATION");
  }

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->generate_global_name(as2_names::topics::sensor_measurements::odom),
      as2_names::topics::sensor_measurements::qos,
      std::bind(&BasicStateEstimator::odomCallback, this, std::placeholders::_1));

  if (ground_truth_) {
    gt_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->generate_global_name(as2_names::topics::ground_truth::pose),
        as2_names::topics::sensor_measurements::qos,
        std::bind(&BasicStateEstimator::gtPoseCallback, this, std::placeholders::_1));

    gt_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        this->generate_global_name(as2_names::topics::ground_truth::twist),
        as2_names::topics::sensor_measurements::qos,
        std::bind(&BasicStateEstimator::gtTwistCallback, this, std::placeholders::_1));
  }

  // if (rectified_localization_) {
  //   rectified_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
  //       this->generate_global_name(FRAME_RECTIFIED_TOPIC),
  //       as2_names::topics::sensor_measurements::qos,
  //       std::bind(&BasicStateEstimator::rectPoseCallback, this, std::placeholders::_1));
  // }

  pose_estimated_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos);
  twist_estimated_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos);

  std::string ns = this->get_namespace();
  // RCLCPP_INFO(get_logger(), "Node namespace: %s", ns.c_str());
  global_ref_frame_ = as2::tf::generateTfName(ns, global_ref_frame);
  map_frame_        = as2::tf::generateTfName(ns, "map");
  odom_frame_       = as2::tf::generateTfName(ns, "odom");
  if (base_frame == "") {
    baselink_frame_ = ns.substr(1, ns.length());
    RCLCPP_WARN(get_logger(), "NO BASE FRAME SPECIFIED , USING DEFAULT: %s",
                baselink_frame_.c_str());
  } else {
    baselink_frame_ = as2::tf::generateTfName(ns, base_frame);
  }

  frame_global_pose_  = global_ref_frame_;
  frame_global_twist_ = baselink_frame_;
}

void BasicStateEstimator::setupTfTree() {
  tf2_fix_transforms_.clear();

  getStartingPose(global_ref_frame_, map_frame_);
  publishStaticTfs();

  // init map_2_odom
  map2odom_tf_.header.frame_id      = map_frame_;
  map2odom_tf_.child_frame_id       = odom_frame_;
  map2odom_tf_.transform.rotation.w = 1.0f;

  // init odom_2_baselink
  odom2baselink_tf_.header.frame_id      = odom_frame_;
  odom2baselink_tf_.child_frame_id       = baselink_frame_;
  odom2baselink_tf_.transform.rotation.w = 1.0f;

  RCLCPP_INFO(get_logger(), "%s -> %s", global_ref_frame_.c_str(), map_frame_.c_str());
  RCLCPP_INFO(get_logger(), "%s -> %s", map2odom_tf_.header.frame_id.c_str(),
              map2odom_tf_.child_frame_id.c_str());
  RCLCPP_INFO(get_logger(), "%s -> %s", odom2baselink_tf_.header.frame_id.c_str(),
              odom2baselink_tf_.child_frame_id.c_str());

  // if (rectified_localization_) {
  //   // map2frame_rectified_tf_.header.frame_id = map_frame_;
  //   frame_rectified_tf_.header.frame_id      = "";
  //   frame_rectified_tf_.child_frame_id       = "";
  //   frame_rectified_tf_.transform.rotation.w = 1.0f;

  //   RCLCPP_INFO(get_logger(), "Expecting rectification info");
  //   // RCLCPP_INFO(get_logger(), "%s -> %s", frame_rectified_tf_.header.frame_id.c_str(),
  //   // frame_rectified_tf_.child_frame_id.c_str());
  // }

  start_run_ = false;
}

void BasicStateEstimator::getStartingPose(const std::string &_global_frame,
                                          const std::string &_map) {
  // TODO: Get starting pose

  // Default
  tf2_fix_transforms_.emplace_back(
      as2::tf::getTransformation(_global_frame, _map, 0, 0, 0, 0, 0, 0));
}

void BasicStateEstimator::updateOdomTfDrift(
    const geometry_msgs::msg::TransformStamped &_odom2baselink,
    const geometry_msgs::msg::TransformStamped &_map2baselink) {
  map2odom_tf_.header.stamp    = _map2baselink.header.stamp;
  map2odom_tf_.header.frame_id = _map2baselink.header.frame_id;
  map2odom_tf_.child_frame_id  = _odom2baselink.header.frame_id;

  map2odom_tf_.transform.translation.x =
      _map2baselink.transform.translation.x - _odom2baselink.transform.translation.x;
  map2odom_tf_.transform.translation.y =
      _map2baselink.transform.translation.y - _odom2baselink.transform.translation.y;
  map2odom_tf_.transform.translation.z =
      _map2baselink.transform.translation.z - _odom2baselink.transform.translation.z;

  // Calculate relative orientation
  tf2::Quaternion map2baselink_orientation(
      _map2baselink.transform.rotation.x, _map2baselink.transform.rotation.y,
      _map2baselink.transform.rotation.z, _map2baselink.transform.rotation.w);
  tf2::Quaternion odom2baselink_orientation(
      _odom2baselink.transform.rotation.x, _odom2baselink.transform.rotation.y,
      _odom2baselink.transform.rotation.z, _odom2baselink.transform.rotation.w);
  tf2::Quaternion map2odom_orientation =
      map2baselink_orientation * tf2::inverse(odom2baselink_orientation);

  map2odom_tf_.transform.rotation.x = map2odom_orientation.x();
  map2odom_tf_.transform.rotation.y = map2odom_orientation.y();
  map2odom_tf_.transform.rotation.z = map2odom_orientation.z();
  map2odom_tf_.transform.rotation.w = map2odom_orientation.w();
}

void BasicStateEstimator::updateRefTfRectification(
    const geometry_msgs::msg::Transform &_frame_rectified_tf,
    const geometry_msgs::msg::Transform &_ref2frame_rectified_tf) {
  // ref2ref_rectified_tf_.transform.translation.x =
  //     _ref2frame_rectified_tf.translation.x - _frame_rectified_tf.translation.x;
  // ref2ref_rectified_tf_.transform.translation.y =
  //     _ref2frame_rectified_tf.translation.y - _frame_rectified_tf.translation.y;
  // ref2ref_rectified_tf_.transform.translation.z =
  //     _ref2frame_rectified_tf.translation.z - _frame_rectified_tf.translation.z;

  // // Calculate relative orientation
  // tf2::Quaternion ref2frame_rectified_orientation(
  //     _ref2frame_rectified_tf.rotation.x, _ref2frame_rectified_tf.rotation.y,
  //     _ref2frame_rectified_tf.rotation.z, _ref2frame_rectified_tf.rotation.w);
  // tf2::Quaternion ref2ref_rectified_(_frame_rectified_tf.rotation.x,
  // _frame_rectified_tf.rotation.y,
  //                                    _frame_rectified_tf.rotation.z,
  //                                    _frame_rectified_tf.rotation.w);
  // tf2::Quaternion ref2ref_rectified_orientation =
  //     ref2frame_rectified_orientation * tf2::inverse(ref2ref_rectified_);

  // ref2ref_rectified_tf_.transform.rotation.x = ref2ref_rectified_orientation.x();
  // ref2ref_rectified_tf_.transform.rotation.y = ref2ref_rectified_orientation.y();
  // ref2ref_rectified_tf_.transform.rotation.z = ref2ref_rectified_orientation.z();
  // ref2ref_rectified_tf_.transform.rotation.w = ref2ref_rectified_orientation.w();
}

geometry_msgs::msg::TransformStamped BasicStateEstimator::calculateLocalization() {
  geometry_msgs::msg::TransformStamped map2baselink;
  if (odom_only_) {
    map2baselink.header.stamp    = odom2baselink_tf_.header.stamp;
    map2baselink.header.frame_id = map_frame_;
    map2baselink.child_frame_id  = odom2baselink_tf_.child_frame_id;
    map2baselink.transform       = odom2baselink_tf_.transform;
  }
  if (ground_truth_) {
    map2baselink.header.stamp            = gt_pose_stamped_.header.stamp;
    map2baselink.child_frame_id          = baselink_frame_;
    map2baselink.transform.translation.x = gt_pose_stamped_.pose.position.x;
    map2baselink.transform.translation.y = gt_pose_stamped_.pose.position.y;
    map2baselink.transform.translation.z = gt_pose_stamped_.pose.position.z;
    map2baselink.transform.rotation.x    = gt_pose_stamped_.pose.orientation.x;
    map2baselink.transform.rotation.y    = gt_pose_stamped_.pose.orientation.y;
    map2baselink.transform.rotation.z    = gt_pose_stamped_.pose.orientation.z;
    map2baselink.transform.rotation.w    = gt_pose_stamped_.pose.orientation.w;

    odom2baselink_tf_ = map2baselink;

    map2baselink.header.frame_id      = map_frame_;
    odom2baselink_tf_.header.frame_id = odom_frame_;
  }

  // if (rectified_localization_)
  // {
  //   frame_rectified_tf_.header.frame_id = rectified_pose_.header.frame_id;
  //   frame_rectified_tf_.transform.translation.x = rectified_pose_.pose.position.x;
  //   frame_rectified_tf_.transform.translation.y = rectified_pose_.pose.position.y;
  //   frame_rectified_tf_.transform.translation.z = rectified_pose_.pose.position.z;
  //   frame_rectified_tf_.transform.rotation.x = rectified_pose_.pose.orientation.x;
  //   frame_rectified_tf_.transform.rotation.y = rectified_pose_.pose.orientation.y;
  //   frame_rectified_tf_.transform.rotation.z = rectified_pose_.pose.orientation.z;
  //   frame_rectified_tf_.transform.rotation.w = rectified_pose_.pose.orientation.w;
  // }

  if (sensor_fusion_) {
    // TODO: SENSOR FUSION
  }
  return map2baselink;
}

void BasicStateEstimator::getGlobalRefState() {
  using namespace std::chrono_literals;
  // Pose in the global frame
  try {
    auto pose_transform =
        tf_buffer_->lookupTransform(frame_global_pose_, baselink_frame_, tf_publish_time_, 50ms);
    global_ref_pose_.header.frame_id    = frame_global_pose_;
    global_ref_pose_.header.stamp       = pose_transform.header.stamp;
    global_ref_pose_.pose.position.x    = pose_transform.transform.translation.x;
    global_ref_pose_.pose.position.y    = pose_transform.transform.translation.y;
    global_ref_pose_.pose.position.z    = pose_transform.transform.translation.z;
    global_ref_pose_.pose.orientation.x = pose_transform.transform.rotation.x;
    global_ref_pose_.pose.orientation.y = pose_transform.transform.rotation.y;
    global_ref_pose_.pose.orientation.z = pose_transform.transform.rotation.z;
    global_ref_pose_.pose.orientation.w = pose_transform.transform.rotation.w;

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform Failure: %s\n",
                ex.what());  // Print exception which was caught
  }

  // Twist in target frame
  if (odom_only_) {
    if (odom_twist_.header.frame_id == frame_global_twist_) {
      global_ref_twist_ = odom_twist_;
    } else {
      try {
        auto twist_transform = tf_buffer_->lookupTransform(
            frame_global_twist_, odom_twist_.header.frame_id, odom_twist_.header.stamp);

        geometry_msgs::msg::Vector3Stamped odom_linear_twist;
        odom_linear_twist.header = odom_twist_.header;
        odom_linear_twist.vector = odom_twist_.twist.linear;

        geometry_msgs::msg::Vector3Stamped global_linear_twist;
        tf2::doTransform(odom_linear_twist, global_linear_twist, twist_transform);

        // TODO: tranform angular velocity
        // geometry_msgs::msg::Vector3Stamped odom_angular_twist;

        global_ref_twist_.header        = global_linear_twist.header;
        global_ref_twist_.twist.linear  = global_linear_twist.vector;
        global_ref_twist_.twist.angular = odom_twist_.twist.angular;

      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform Failure: %s\n",
                    ex.what());  // Print exception which was caught
      }

      // global_ref_twist.header.frame_id = odom_twist_.header.frame_id;
      // global_ref_twist.header.stamp    = odom_twist_.header.stamp;
      // global_ref_twist.twist.angular   = odom_twist_.twist.angular;
      // tf2::Quaternion orientation(global_ref_pose.orientation.x, global_ref_pose.orientation.y,
      //                             global_ref_pose.orientation.z, global_ref_pose.orientation.w);

      // // odom2baselink_tf_.transform.rotation.x, odom2baselink_tf_.transform.rotation.y,
      // // odom2baselink_tf_.transform.rotation.z, odom2baselink_tf_.transform.rotation.w);

      // Eigen::Vector3d odom_linear_twist(odom_twist_.twist.linear.x, odom_twist_.twist.linear.y,
      //                                   odom_twist_.twist.linear.z);
      // Eigen::Vector3d global_linear_twist =
      //     as2::frame::convertFLUtoENU(orientation, odom_linear_twist);
      // global_ref_twist.twist.linear.x = global_linear_twist.x();
      // global_ref_twist.twist.linear.y = global_linear_twist.y();
      // global_ref_twist.twist.linear.z = global_linear_twist.z();
    }
  }

  if (ground_truth_) {
    global_ref_twist_ = gt_twist_stamped_;
  }

  if (sensor_fusion_) {  // TODO: Sensor fusion
  }
}

// PUBLISH //

void BasicStateEstimator::publishTfs() {
  // map2odom_tf_.header.stamp = tf_publish_time_;
  tf_broadcaster_->sendTransform(map2odom_tf_);
  // odom2baselink_tf_.header.stamp = tf_publish_time_;
  tf_broadcaster_->sendTransform(odom2baselink_tf_);
  // if (rectified_localization_ &&
  //     (ref2ref_rectified_tf_.header.frame_id != "" && ref2ref_rectified_tf_.child_frame_id !=
  //     "")) {
  //   ref2ref_rectified_tf_.header.stamp = tf_publish_time_;
  //   tf_broadcaster_->sendTransform(ref2ref_rectified_tf_);
  // }
  tf_publish_time_ = odom2baselink_tf_.header.stamp;
}

void BasicStateEstimator::publishStaticTfs() {
  for (geometry_msgs::msg::TransformStamped &transform : tf2_fix_transforms_) {
    tfstatic_broadcaster_->sendTransform(transform);
  }
}

void BasicStateEstimator::publishStateEstimation() {
  // rclcpp::Time timestamp = this->get_clock()->now();
  rclcpp::Time timestamp = tf_publish_time_;
  pose_estimated_pub_->publish(generatePoseStampedMsg(timestamp));
  twist_estimated_pub_->publish(generateTwistStampedMsg(timestamp));
}

geometry_msgs::msg::PoseStamped BasicStateEstimator::generatePoseStampedMsg(
    const rclcpp::Time &_timestamp) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  // pose_stamped.header.stamp     = _timestamp;
  pose_stamped.header.stamp     = _timestamp;
  pose_stamped.header.frame_id  = global_ref_pose_.header.frame_id;
  pose_stamped.pose.position    = global_ref_pose_.pose.position;
  pose_stamped.pose.orientation = global_ref_pose_.pose.orientation;
  return pose_stamped;
}

geometry_msgs::msg::TwistStamped BasicStateEstimator::generateTwistStampedMsg(
    const rclcpp::Time &_timestamp) {
  geometry_msgs::msg::TwistStamped twist_stamped;
  twist_stamped.header.stamp    = _timestamp;
  twist_stamped.header.frame_id = global_ref_twist_.header.frame_id;  // TODO:Review ref frame
  twist_stamped.twist.linear    = global_ref_twist_.twist.linear;
  twist_stamped.twist.angular   = global_ref_twist_.twist.angular;
  return twist_stamped;
}

// CALLBACKS //

void BasicStateEstimator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr _msg) {
  // rclcpp::Time timestamp = this->get_clock()->now();
  odom2baselink_tf_.header.frame_id         = _msg->header.frame_id;
  odom2baselink_tf_.child_frame_id          = _msg->child_frame_id;
  odom2baselink_tf_.header.stamp            = _msg->header.stamp;
  odom2baselink_tf_.transform.translation.x = _msg->pose.pose.position.x;
  odom2baselink_tf_.transform.translation.y = _msg->pose.pose.position.y;
  odom2baselink_tf_.transform.translation.z = _msg->pose.pose.position.z;
  odom2baselink_tf_.transform.rotation.x    = _msg->pose.pose.orientation.x;
  odom2baselink_tf_.transform.rotation.y    = _msg->pose.pose.orientation.y;
  odom2baselink_tf_.transform.rotation.z    = _msg->pose.pose.orientation.z;
  odom2baselink_tf_.transform.rotation.w    = _msg->pose.pose.orientation.w;

  odom_twist_.header.frame_id = _msg->child_frame_id;
  odom_twist_.header.stamp    = _msg->header.stamp;
  odom_twist_.twist.linear    = _msg->twist.twist.linear;
  odom_twist_.twist.angular   = _msg->twist.twist.angular;

  last_info_time_ = _msg->header.stamp;
  start_run_      = true;
}

void BasicStateEstimator::gtPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg) {
  gt_pose_stamped_.header = _msg->header;
  gt_pose_stamped_.pose   = _msg->pose;

  last_info_time_ = _msg->header.stamp;
  start_run_      = true;
}

void BasicStateEstimator::gtTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr _msg) {
  gt_twist_stamped_.header = _msg->header;
  gt_twist_stamped_.twist  = _msg->twist;

  last_info_time_ = _msg->header.stamp;
  start_run_      = true;
}

void BasicStateEstimator::rectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg) {
  // frame_rectified_tf_.header.frame_id         = _msg->header.frame_id;
  // std::string rect_frame                      = frame_rectified_tf_.header.frame_id;
  // frame_rectified_tf_.child_frame_id          = baselink_frame_;
  // frame_rectified_tf_.transform.translation.x = _msg->pose.position.x;
  // frame_rectified_tf_.transform.translation.y = _msg->pose.position.y;
  // frame_rectified_tf_.transform.translation.z = _msg->pose.position.z;
  // frame_rectified_tf_.transform.rotation.x    = _msg->pose.orientation.x;
  // frame_rectified_tf_.transform.rotation.y    = _msg->pose.orientation.y;
  // frame_rectified_tf_.transform.rotation.z    = _msg->pose.orientation.z;
  // frame_rectified_tf_.transform.rotation.w    = _msg->pose.orientation.w;

  // RCLCPP_INFO_ONCE(get_logger(), "Received pose %s -> %s",
  //                  frame_rectified_tf_.header.frame_id.c_str(),
  //                  frame_rectified_tf_.child_frame_id.c_str());
  // RCLCPP_INFO_ONCE(get_logger(), "Updated: Added %s",
  // frame_rectified_tf_.header.frame_id.c_str()); start_run_ = true;
}

void BasicStateEstimator::cleanupNode(){
    // TODO: Cleanup Node
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn BasicStateEstimator::on_configure(const rclcpp_lifecycle::State &_state) {
  // Set subscriptions, publishers, services, actions, etc. here.
  setupNode();

  return CallbackReturn::SUCCESS;
};

CallbackReturn BasicStateEstimator::on_activate(const rclcpp_lifecycle::State &_state) {
  // Set parameters?
  setupTfTree();

  return CallbackReturn::SUCCESS;
};

CallbackReturn BasicStateEstimator::on_deactivate(const rclcpp_lifecycle::State &_state) {
  // Clean up subscriptions, publishers, services, actions, etc. here.
  cleanupNode();

  return CallbackReturn::SUCCESS;
};

CallbackReturn BasicStateEstimator::on_shutdown(const rclcpp_lifecycle::State &_state) {
  // Clean other resources here.

  return CallbackReturn::SUCCESS;
};
