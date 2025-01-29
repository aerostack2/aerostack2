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
* @file as2_state_estimator.hpp
*
* An state estimation server for AeroStack2
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#ifndef AS2_STATE_ESTIMATOR__AS2_STATE_ESTIMATOR_HPP_
#define AS2_STATE_ESTIMATOR__AS2_STATE_ESTIMATOR_HPP_

#include <array>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
#include <string>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <as2_core/names/topics.hpp>
#include <as2_core/node.hpp>
#include <as2_core/utils/frame_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>

#include "as2_state_estimator/plugin_base.hpp"
namespace as2_state_estimator
{

using StateComponent = std::variant<
  geometry_msgs::msg::PoseWithCovariance,
  geometry_msgs::msg::TwistWithCovariance>;


class StateEstimator : public as2::Node
{
public:
  explicit StateEstimator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StateEstimator() {}

  bool filterTransformRule(const geometry_msgs::msg::TransformStamped & transform)
  {
    if (transform.header.frame_id == earth_frame_id_ && transform.child_frame_id == map_frame_id_) {
      return false;
    }
    if (transform.header.frame_id == map_frame_id_ && transform.child_frame_id == odom_frame_id_) {
      return false;
    }
    if (transform.header.frame_id == odom_frame_id_ && transform.child_frame_id == base_frame_id_) {
      return false;
    }
    return true;
  }

private:
  using StateEstimatorBase = as2_state_estimator_plugin_base::StateEstimatorBase;
  std::shared_ptr<pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>>
  loader_;
  // std::shared_ptr<as2_state_estimator_plugin_base::StateEstimatorBase> plugin_ptr_;

  std::unordered_map<std::string, std::shared_ptr<StateEstimatorBase>> plugins_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_broadcaster_;
  // std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  std::unordered_map<std::string,
    std::shared_ptr<StateEstimatorInterface>> state_estimator_interfaces_;
  std::unordered_map<std::string, std::shared_ptr<as2::tf::TfHandler>> tf_handlers_;

  void declareRosInterfaces()
  {
    // tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tfstatic_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos);
  }

  bool loadPlugin(const std::string & plugin_name);


  void readParameters()
  {
    // TODO(miferco97): Add try to get_declared_parameter for
    // avoid needing to always declare the standard parameters in the launch file

    // node_ptr_->declare_parameter<std::string>("base_frame", "base_link");
    // node_ptr_->declare_parameter<std::string>("global_ref_frame", "earth");
    // node_ptr_->declare_parameter<std::string>("odom_frame", "odom");
    // node_ptr_->declare_parameter<std::string>("map_frame", "map");

    this->get_parameter("base_frame", base_frame_id_);
    this->get_parameter("global_ref_frame", earth_frame_id_);
    this->get_parameter("odom_frame", odom_frame_id_);
    this->get_parameter("map_frame", map_frame_id_);

    base_frame_id_ = as2::tf::generateTfName(this, base_frame_id_);
    odom_frame_id_ = as2::tf::generateTfName(this, odom_frame_id_);
    map_frame_id_ = as2::tf::generateTfName(this, map_frame_id_);

    RCLCPP_INFO(this->get_logger(), "Frame names:");
    RCLCPP_INFO(this->get_logger(), "\tEarth frame: %s", earth_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "\tMap frame: %s", map_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "\tOdom frame: %s", odom_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "\tBase frame: %s", base_frame_id_.c_str());
  }

  friend class MetacontrollerInterface;

private:
  std::string earth_frame_id_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  std::string map_frame_id_;
  std::array<std::vector<std::string>, 4> authorithed_plugins_;

  void printAvailablePlugins()
  {
    // log the plugins that are available for each type

    for (int i = 0; i < 4; i++) {
      RCLCPP_INFO(
        this->get_logger(), "Plugins available for type %s", as2_state_estimator::TransformInformatonTypeToString(
          static_cast<as2_state_estimator::TransformInformatonType>(i)).c_str());
      for (const auto & plugin : authorithed_plugins_[i]) {
        RCLCPP_INFO(this->get_logger(), "\t%s", plugin.c_str());
      }
    }

  }

  void registerPlugin(const std::string & plugin_name)
  {
    // find if the plugin is already Loaded
    if (plugins_.find(plugin_name) == plugins_.end()) {
      RCLCPP_WARN(this->get_logger(), "Plugin %s is not loaded", plugin_name.c_str());
      return;
    }
    auto type_list = plugins_[plugin_name]->getTransformationTypesAvailable();
    for (const auto & type : type_list) {
      authorithed_plugins_[static_cast<int>(type)].push_back(plugin_name);
    }
  }

  void processStateComponent(
    const std::string & authority, const StateComponent & component,
    const as2_state_estimator::TransformInformatonType & type,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false)
  {
    // assert if the authority is correct for the type
    if (!checkSourceAuthority(authority, type)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Authority %s is not allowed to publish in type %s", authority.c_str(),
        as2_state_estimator::TransformInformatonTypeToString(type).c_str());
      return;
    }

    std::visit(
      [this, &authority, &type, &stamp, &is_static](auto && arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, geometry_msgs::msg::PoseWithCovariance>) {
          RCLCPP_INFO(this->get_logger(), "Processing Pose, authority: %s", authority.c_str());
          processPose(authority, arg, type, stamp, is_static);

        } else if constexpr (std::is_same_v<T, geometry_msgs::msg::TwistWithCovariance>) {
          RCLCPP_INFO(this->get_logger(), "Processing Twist, authority: %s", authority.c_str());
          processTwist(authority, arg, stamp);
        }
      }, component);
  }

  bool checkSourceAuthority(const std::string & authority, const TransformInformatonType & type)
  {
    auto plugin_list = authorithed_plugins_[static_cast<int>(type)];
    if (std::find(plugin_list.begin(), plugin_list.end(), authority) == plugin_list.end()) {
      return false;
    }
    return true;
  }

  // void processEarthToMap(
  //   const std::string & authority,
  //   const geometry_msgs::msg::PoseWithCovariance & msg,
  //   const builtin_interfaces::msg::Time & stamp,
  //   bool is_static = false);
  // void processMapToOdom(
  //   const std::string & authority,
  //   const geometry_msgs::msg::PoseWithCovariance & msg,
  //   const builtin_interfaces::msg::Time & stamp,
  //   bool is_static = false);
  // void processOdomToBase(
  //   const std::string & authority,
  //   const geometry_msgs::msg::PoseWithCovariance & msg,
  //   const builtin_interfaces::msg::Time & stamp);

  std::pair<std::string, std::string> getFramesFromType(
    as2_state_estimator::TransformInformatonType type)
  {
    std::string parent_frame, child_frame;
    switch (type) {
      case as2_state_estimator::TransformInformatonType::EARTH_TO_MAP:
        parent_frame = earth_frame_id_;
        child_frame = map_frame_id_;
        break;
      case as2_state_estimator::TransformInformatonType::MAP_TO_ODOM:
        parent_frame = map_frame_id_;
        child_frame = odom_frame_id_;
        break;
      case as2_state_estimator::TransformInformatonType::ODOM_TO_BASE:
        parent_frame = odom_frame_id_;
        child_frame = base_frame_id_;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown transform type");
        return {"", ""};
    }
    return {parent_frame, child_frame};
  }


  void processPose(
    const std::string & authority,
    const geometry_msgs::msg::PoseWithCovariance & msg,
    const as2_state_estimator::TransformInformatonType & type,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false);

  void processTwist(
    const std::string & authority,
    const geometry_msgs::msg::TwistWithCovariance & msg,
    const builtin_interfaces::msg::Time & stamp);

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const builtin_interfaces::msg::Time & stamp,
    bool is_static = false);

  void publishTransform(
    const geometry_msgs::msg::PoseWithCovariance & pose, const std::string & parent_frame,
    const std::string & child_frame, const builtin_interfaces::msg::Time & stamp,
    bool is_static = false);


  void publish_initial_transforms();

  void publishTwist(
    const geometry_msgs::msg::TwistWithCovariance & twist,
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = stamp;
    twist_msg.header.frame_id = base_frame_id_;
    twist_msg.twist = twist.twist;
    twist_pub_->publish(twist_msg);
    publishPoseInEarthFrame(stamp);
  }

  void publishPoseInEarthFrame(
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    tf2::Transform earth_to_base = getEarthToMapTransform() * getMapToOdomTransform() *
      getOdomToBaseLinkTransform();
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = earth_frame_id_;
    auto pose = tf2::toMsg(earth_to_base);
    pose_msg.pose.position.x = pose.translation.x;
    pose_msg.pose.position.y = pose.translation.y;
    pose_msg.pose.position.z = pose.translation.z;
    pose_msg.pose.orientation = pose.rotation;
    pose_pub_->publish(pose_msg);
  }


  inline tf2::Transform & getEarthToMapTransform()
  {
    return transforms_[as2_state_estimator::TransformInformatonType::EARTH_TO_MAP];
  }
  inline tf2::Transform & getMapToOdomTransform()
  {
    return transforms_[as2_state_estimator::TransformInformatonType::MAP_TO_ODOM];
  }
  inline tf2::Transform & getOdomToBaseLinkTransform()
  {
    return transforms_[as2_state_estimator::TransformInformatonType::ODOM_TO_BASE];
  }

  std::array<tf2::Transform,
    3> transforms_ =
  {tf2::Transform::getIdentity(), tf2::Transform::getIdentity(), tf2::Transform::getIdentity()};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  /**
   * @brief Modify the node options to allow undeclared parameters
   */
  static rclcpp::NodeOptions get_modified_options(const rclcpp::NodeOptions & options);
};  // class StateEstimator
}  // namespace as2_state_estimator

#endif  // AS2_STATE_ESTIMATOR__AS2_STATE_ESTIMATOR_HPP_
