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
 *  \file       motion_reference_bus.cpp
 *  \brief      Implementation of the resources shared by the handlers of one node
 *  \authors    Rafael Pérez Seguí
 ********************************************************************************/

#include "motion_reference_bus.hpp"

#include <memory>
#include <string>

#include "as2_core/names/topics.hpp"

namespace as2
{
namespace motionReferenceHandlers
{

std::map<MotionReferenceBus::Key, std::weak_ptr<MotionReferenceBus>>
MotionReferenceBus::registry_;
std::mutex MotionReferenceBus::registry_mutex_;

std::shared_ptr<MotionReferenceBus> MotionReferenceBus::get(
  as2::Node * node, const std::string & ns)
{
  const Key key{node, ns};
  std::lock_guard<std::mutex> lock(registry_mutex_);

  auto it = registry_.find(key);
  if (it != registry_.end()) {
    if (auto existing = it->second.lock()) {
      return existing;
    }
  }
  // make_shared is not usable here: the constructor is private
  std::shared_ptr<MotionReferenceBus> created(new MotionReferenceBus(node, ns));
  registry_[key] = created;
  return created;
}

MotionReferenceBus::~MotionReferenceBus()
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  registry_.erase(Key{node_ptr_, namespace_});
}

MotionReferenceBus::MotionReferenceBus(as2::Node * node, const std::string & ns)
: node_ptr_(node), namespace_(ns)
{
  use_actuator_commands_ = node_ptr_->getParameter<bool>("use_actuator_commands", false);

  // Publishers. Commands go to the platform, or to the motion controller
  const std::string trajectory_topic = use_actuator_commands_ ?
    as2_names::topics::actuator_command::trajectory :
    as2_names::topics::motion_reference::trajectory;
  const std::string pose_topic = use_actuator_commands_ ?
    as2_names::topics::actuator_command::pose : as2_names::topics::motion_reference::pose;
  const std::string twist_topic = use_actuator_commands_ ?
    as2_names::topics::actuator_command::twist : as2_names::topics::motion_reference::twist;
  const std::string thrust_topic = use_actuator_commands_ ?
    as2_names::topics::actuator_command::thrust : as2_names::topics::motion_reference::thrust;
  const rclcpp::QoS command_qos = use_actuator_commands_ ?
    as2_names::topics::actuator_command::qos : as2_names::topics::motion_reference::qos;

  command_traj_pub_ = node_ptr_->create_publisher<as2_msgs::msg::TrajectorySetpoints>(
    namespace_ + trajectory_topic, command_qos);

  command_pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
    namespace_ + pose_topic, command_qos);

  command_twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
    namespace_ + twist_topic, command_qos);

  command_thrust_pub_ = node_ptr_->create_publisher<as2_msgs::msg::Thrust>(
    namespace_ + thrust_topic, command_qos);

  // Subscriber to the current control mode, from the platform or the controller
  if (use_actuator_commands_) {
    platform_info_sub_ = node_ptr_->create_subscription<as2_msgs::msg::PlatformInfo>(
      namespace_ + as2_names::topics::platform::info, rclcpp::QoS(1),
      [this](const as2_msgs::msg::PlatformInfo::SharedPtr msg) {
        current_mode_ = msg->current_control_mode;
      });
  } else {
    controller_info_sub_ = node_ptr_->create_subscription<as2_msgs::msg::ControllerInfo>(
      namespace_ + as2_names::topics::controller::info, rclcpp::QoS(1),
      [this](const as2_msgs::msg::ControllerInfo::SharedPtr msg) {
        current_mode_ = msg->input_control_mode;
      });
  }
}

}    // namespace motionReferenceHandlers
}  // namespace as2
