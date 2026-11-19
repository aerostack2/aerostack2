// Copyright 2026 Vishwa
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
//    * Neither the name of the copyright holder nor the names of its
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
* @file flight_governor.hpp
*
* ROS 2 node that governs motion references between the behaviors and the
* motion controller.
*
* @author Vishwa
*/

#ifndef AS2_FLIGHT_GOVERNOR__FLIGHT_GOVERNOR_HPP_
#define AS2_FLIGHT_GOVERNOR__FLIGHT_GOVERNOR_HPP_

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "as2_core/node.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/alert_event.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/controller_info.hpp"
#include "as2_msgs/msg/governor_verdict.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "as2_flight_governor/governor_core.hpp"

namespace as2_flight_governor
{

/// Topic prefix for the governed references the controller must listen to.
constexpr char kOutputPrefix[] = "flight_governor/";

/**
 * Subscribes to `motion_reference/<type>` (what the behaviors publish), applies
 * the governor according to the controller's input control mode, and publishes
 * the result on `flight_governor/motion_reference/<type>`, which the motion
 * controller reads when launched with `use_flight_governor:=true`.
 *
 * Every reference produces a `flight_governor/verdict` message.
 */
class FlightGovernor : public as2::Node
{
public:
  explicit FlightGovernor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  const Governor & governor() const {return *governor_;}

  /// Smallest speed limit ever written: 0 means "unlimited" to the controller.
  static constexpr double kMinSpeedLimit = 0.01;
  /// Written for an unlimited axis when another axis is limited, because a
  /// single 0 would disable the limits on every axis.
  static constexpr double kUnlimitedSpeed = 1000.0;

private:
  void stateCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void controllerInfoCallback(const as2_msgs::msg::ControllerInfo::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void trajectoryCallback(const as2_msgs::msg::TrajectorySetpoints::SharedPtr msg);
  void thrustCallback(const as2_msgs::msg::Thrust::SharedPtr msg);
  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void checkWiring();

  void governPositionPose(const geometry_msgs::msg::PoseStamped & msg);
  void governSpeedLimit(const geometry_msgs::msg::TwistStamped & msg);
  void governVelocity(const geometry_msgs::msg::TwistStamped & msg);

  /// Forward a reference this governor cannot interpret in the current mode.
  template<typename MsgT, typename PubT>
  void passThrough(
    const MsgT & msg, const PubT & pub, const std::string & type,
    const std::string & why);

  /// Publish the verdict, and raise the alert when a veto starts.
  void report(
    const Decision & decision, const std_msgs::msg::Header & header,
    const std::string & type, bool governed, bool sent);

  double nowSeconds();
  double stampSeconds(const builtin_interfaces::msg::Time & stamp) const;
  bool toPolicyFrame(geometry_msgs::msg::PoseStamped & msg);
  bool toPolicyFrame(geometry_msgs::msg::TwistStamped & msg);

  std::unique_ptr<Governor> governor_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  std::string policy_frame_;
  int veto_alert_ = as2_msgs::msg::AlertEvent::INFO_ALERT;
  bool was_vetoing_ = false;

  std::optional<as2_msgs::msg::ControlMode> input_mode_;
  /// Last vehicle pose in the policy frame, for the hold orientation.
  std::optional<geometry_msgs::msg::PoseStamped> last_pose_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr state_sub_;
  rclcpp::Subscription<as2_msgs::msg::ControllerInfo>::SharedPtr controller_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::TrajectorySetpoints>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<as2_msgs::msg::Thrust>::SharedPtr thrust_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<as2_msgs::msg::TrajectorySetpoints>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr thrust_pub_;
  rclcpp::Publisher<as2_msgs::msg::GovernorVerdict>::SharedPtr verdict_pub_;
  rclcpp::Publisher<as2_msgs::msg::AlertEvent>::SharedPtr alert_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr wiring_timer_;
};

}  // namespace as2_flight_governor

#endif  // AS2_FLIGHT_GOVERNOR__FLIGHT_GOVERNOR_HPP_
