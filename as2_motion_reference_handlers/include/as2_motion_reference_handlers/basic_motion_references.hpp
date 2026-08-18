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
 *  \file       basic_motion_references.hpp
 *  \brief      Virtual class for basic motion references headers
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#ifndef AS2_MOTION_REFERENCE_HANDLERS__BASIC_MOTION_REFERENCES_HPP_
#define AS2_MOTION_REFERENCE_HANDLERS__BASIC_MOTION_REFERENCES_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <as2_msgs/msg/control_mode.hpp>
#include <as2_msgs/msg/thrust.hpp>
#include <as2_msgs/msg/controller_info.hpp>
#include <as2_msgs/msg/platform_info.hpp>
#include <as2_msgs/msg/trajectory_setpoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"

namespace as2
{
namespace motionReferenceHandlers
{
class MotionReferenceBus;

/**
 * @brief Base class of the motion reference handlers, which publish motion
 * references and negotiate the control mode they need.
 *
 * References are sent to the motion controller, unless the ROS 2 parameter
 * use_actuator_commands of the node is true: then they are sent straight to the
 * aerial platform, which is also the one the control mode is negotiated with.
 * The platform must support the control modes the references need, since there
 * is no controller in between to synthesize them.
 */
class BasicMotionReferenceHandler
{
public:
  /**
   * @brief Construct the handler, reading use_actuator_commands from the node
   * and creating the publishers and the control mode subscription it selects.
   *
   * @param as2_ptr Node the references are published from.
   * @param ns      Namespace of the drone. Empty to use the node namespace.
   */
  explicit BasicMotionReferenceHandler(as2::Node * as2_ptr, const std::string & ns = "");

  /**
   * @brief Destroy the handler, releasing its publishers and subscription.
   */
  ~BasicMotionReferenceHandler();

protected:
  as2::Node * node_ptr_;
  std::string namespace_;

  as2_msgs::msg::TrajectorySetpoints command_trajectory_msg_;
  geometry_msgs::msg::PoseStamped command_pose_msg_;
  geometry_msgs::msg::TwistStamped command_twist_msg_;
  as2_msgs::msg::Thrust command_thrust_msg_;

  as2_msgs::msg::ControlMode desired_control_mode_;

  /**
   * @brief Send the current thrust reference, settling the control mode first.
   *
   * Unlike the other references, thrust carries no frame, so there is no frame check.
   *
   * @return true if the reference was published.
   */
  bool sendThrustCommand();

  /**
   * @brief Send the current pose reference, settling the control mode first.
   *
   * @return true if the reference was published. False if it has no frame_id.
   */
  bool sendPoseCommand();

  /**
   * @brief Send the current twist reference, settling the control mode first.
   *
   * @return true if the reference was published. False if it has no frame_id.
   */
  bool sendTwistCommand();

  /**
   * @brief Send the current trajectory reference, settling the control mode first.
   *
   * @return true if the reference was published. False if it has no frame_id.
   */
  bool sendTrajectoryCommand();

  /**
   * @brief Negotiate the desired control mode if the active one does not match it.
   *
   * Hovering does not need to be settled again, whatever its yaw mode is.
   *
   * @return true if the desired control mode is the active one.
   */
  bool checkMode();

private:
  /**
   * @brief Check that a reference carries the frame its data is expressed in.
   *
   * @param frame_id  Frame id of the reference message.
   * @param reference Name of the reference, for the error message.
   * @return true if the reference can be sent.
   */
  bool checkFrameId(const std::string & frame_id, const std::string & reference);

  /**
   * @brief Publishers, control mode subscription and active mode, shared by every handler
   * built on the same node and namespace.
   *
   * Several handlers on one node talk to the same controller or platform, so they must
   * share one set of publishers and, above all, one view of the active control mode: if
   * each kept its own, a mode settled by one handler would be unknown to the others.
   * Defined in the implementation, since it is not part of the public interface.
   */
  std::shared_ptr<MotionReferenceBus> shared_;

  /**
   * @brief Negotiate a control mode with the motion controller, or with the
   * aerial platform when use_actuator_commands is true.
   *
   * @param mode Control mode to request.
   * @return true if the mode was accepted.
   */
  bool setMode(const as2_msgs::msg::ControlMode & mode);
};

}    // namespace motionReferenceHandlers
}  // namespace as2

#endif  // AS2_MOTION_REFERENCE_HANDLERS__BASIC_MOTION_REFERENCES_HPP_
