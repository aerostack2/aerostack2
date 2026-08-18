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
 *  \file       motion_reference_bus.hpp
 *  \brief      Resources shared by the motion reference handlers of one node
 *  \authors    Rafael Pérez Seguí
 ********************************************************************************/

#ifndef MOTION_REFERENCE_BUS_HPP_
#define MOTION_REFERENCE_BUS_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <as2_msgs/msg/control_mode.hpp>
#include <as2_msgs/msg/controller_info.hpp>
#include <as2_msgs/msg/platform_info.hpp>
#include <as2_msgs/msg/thrust.hpp>
#include <as2_msgs/msg/trajectory_setpoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"

namespace as2
{
namespace motionReferenceHandlers
{

/**
 * @brief Publishers, control mode subscription and active mode of one (node, namespace).
 *
 * Several handlers on one node talk to the same controller or platform, so they must share
 * one set of publishers and, above all, one view of the active control mode: if each kept
 * its own, a mode settled by one handler would be unknown to the others until the next
 * info message arrived.
 *
 * Handlers obtain it through get(), which hands out the existing bus when another handler
 * of the same node and namespace already built it. The registry holds weak references, so
 * the resources die with the last handler using them rather than living to the end of the
 * process.
 *
 * Not part of the public interface of the package: it lives next to the implementation.
 */
class MotionReferenceBus
{
public:
  /**
   * @brief Get the bus of a (node, namespace), building it on first use.
   *
   * @param node Node the references are published from.
   * @param ns Namespace prefix already resolved, empty for the node's own namespace.
   * @return Bus, shared with any other handler of the same node and namespace.
   */
  static std::shared_ptr<MotionReferenceBus> get(as2::Node * node, const std::string & ns);

  /**
   * @brief Destroy the bus and drop its entry from the registry.
   */
  ~MotionReferenceBus();

  /**
   * @brief Node the references are published from.
   */
  as2::Node * node_ptr_;

  /**
   * @brief Namespace prefix prepended to every topic, empty for the node's own namespace.
   */
  std::string namespace_;

  /**
   * @brief References go straight to the platform, bypassing the motion controller.
   */
  bool use_actuator_commands_ = false;

  /**
   * @brief Mode the controller, or the platform, reports as active.
   */
  as2_msgs::msg::ControlMode current_mode_;

  rclcpp::Publisher<as2_msgs::msg::TrajectorySetpoints>::SharedPtr command_traj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_twist_pub_;
  rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr command_thrust_pub_;

private:
  using Key = std::pair<as2::Node *, std::string>;

  /**
   * @brief Build the publishers and the control mode subscription of a (node, namespace).
   *
   * @param node Node the references are published from.
   * @param ns Namespace prefix already resolved.
   */
  MotionReferenceBus(as2::Node * node, const std::string & ns);

  // Only one of the two is created, depending on use_actuator_commands_
  rclcpp::Subscription<as2_msgs::msg::ControllerInfo>::SharedPtr controller_info_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;

  static std::map<Key, std::weak_ptr<MotionReferenceBus>> registry_;
  static std::mutex registry_mutex_;
};

}    // namespace motionReferenceHandlers
}  // namespace as2

#endif  // MOTION_REFERENCE_BUS_HPP_
