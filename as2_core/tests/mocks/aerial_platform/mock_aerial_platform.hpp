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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
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
 * @file mock_aerial_platform.hpp
 *
 * Class to test the aerial platform node
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef AS2_CORE__TESTS__MOCKS__AERIAL_PLATFORM__MOCK_AERIAL_PLATFORM_HPP_
#define AS2_CORE__TESTS__MOCKS__AERIAL_PLATFORM__MOCK_AERIAL_PLATFORM_HPP_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "as2_msgs/srv/set_platform_state_machine_event.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_point.hpp"
#include "as2_msgs/msg/alert_event.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/utils/control_mode_utils.hpp"

namespace as2
{

namespace mock
{

/**
 * @class PlatformMockNode to test aerostack2 aerial platform node
*/
class PlatformMockNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new PlatformMockNode object
   *
   * @param name_space namespace of the drone
   */
  explicit PlatformMockNode(const std::string & name_space);

  /**
   * @brief Destroy the PlatformMockNode object
  */
  ~PlatformMockNode();

  // Services clients

  /**
   * @brief Set the Arming State of the drone
   *
   * @param arm true to arm the drone, false to disarm
   * @return rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture
   */
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture setArmingStateSrvCall(bool arm);

  /**
   * @brief Set the Offboard Control of the drone
   *
   * @param offboard true to enable offboard control, false to disable
   * @return rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture
   */
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture setOffboardControlSrvCall(bool offboard);

  /**
   * @brief Set the Platform State Machine Event
   *
   * @param request_state state to set
   * @return rclcpp::Client<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedFuture
   */
  rclcpp::Client<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedFuture
  setPlatformStateMachineEventSrvCall(as2_msgs::msg::PlatformStateMachineEvent request_state);

  /**
   * @brief Set the Control Mode of the drone
   *
   * @param control_mode control mode to set
   * @return rclcpp::Client<as2_msgs::srv::SetControlMode>::SharedFuture
   */
  rclcpp::Client<as2_msgs::srv::SetControlMode>::SharedFuture setControlModeSrvCall(
    as2_msgs::msg::ControlMode control_mode);

  /**
   * @brief Takeoff the drone
   *
   * @return rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture
   */
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture takeoffSrvCall();

  /**
   * @brief Land the drone
   *
   * @return rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture
   */
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture landSrvCall();

  // Utils methods

  /**
   * @brief Takeoff the drone.
   * 1. Arm the drone
   * 2. Set offboard control
   * 3. Set taking off state
   * 4. Send taking off command
   *
   * @param spin_executor true to spin the executor, false to wait
   * @return true if the drone is taking off, false otherwise
   */
  bool takeoffPlatform(const bool spin_executor = true);

  /**
   * @brief Land the drone.
   * 1. Set landing state
   * 2. Send landing command
   * 3. Send disarm command
   *
   * @param spin_executor true to spin the executor, false to wait
   * @return true if the drone is landed, false otherwise
  */
  bool landPlatform(const bool spin_executor = true);

  /**
   * @brief Send taking off command for platform takeoff
   *
   * @param spin_executor true to spin the executor, false to wait
   * @return true if the drone is taking off, false otherwise
   */
  bool takeoff(const bool spin_executor = true);

  /**
   * @brief Send landing command for platform landing
   *
   * @param spin_executor true to spin the executor, false to wait
   * @return true if the drone is landed, false otherwise
   */
  bool land(const bool spin_executor = true);

  /**
   * @brief Set the Arming State of the drone
   *
   * @param arm true to arm the drone, false to disarm
   * @param spin_executor true to spin the executor, false to wait
   * @return true if the drone is armed, false otherwise
   */
  bool setArmingState(const bool arm, const bool spin_executor = true);

  /**
   * @brief Set the Offboard Control of the drone
   *
   * @param spin_executor true to spin the executor, false to wait
   * @return true if the offboard control is enabled, false otherwise
   */
  bool setOffboardControl(const bool offboard, const bool spin_executor = true);

  /**
   * @brief Set the Platform State Machine Event
   *
   * @param state_machine_event state machine event to set
   * @param spin_executor true to spin the executor, false to wait
   * @return true if the control mode is set, false otherwise
   */
  bool setPlatformStateMachineEvent(
    const as2_msgs::msg::PlatformStateMachineEvent & state_machine_event,
    const bool spin_executor = true);

  /**
   * @brief Set the Control Mode of the drone
   *
   * @param control_mode control mode to set
   * @param spin_executor true to spin the executor, false to wait
   * @return true if the control mode is set, false otherwise
   */
  bool setControlMode(
    const as2_msgs::msg::ControlMode & control_mode,
    const bool spin_executor = true);

  /**
   * @brief Get the Time object
   *
   * @return rclcpp::Time current time
   */
  rclcpp::Time getTime();

  /**
   * @brief Get platform info
   *
   * @return const as2_msgs::msg::PlatformInfo& platform info
   */
  const as2_msgs::msg::PlatformInfo & getPlatformInfo() const;

  /**
   * @brief Get current ground thruth pose
   *
   * @return const geometry_msgs::msg::PoseStamped& current pose
   */
  const geometry_msgs::msg::PoseStamped & getGroundThruthPose() const;

  /**
   * @brief Get current ground thruth twist
   *
   * @return const geometry_msgs::msg::TwistStamped& current twist
   */
  const geometry_msgs::msg::TwistStamped & getGroundThruthTwist() const;

  /**
   * @brief Get current odometry
   *
   * @return const nav_msgs::msg::Odometry& current odometry
   */
  const nav_msgs::msg::Odometry & getOdometry() const;

  /**
   * @brief Get current gps
   *
   * @return const sensor_msgs::msg::NavSatFix& current gps
   */
  const sensor_msgs::msg::NavSatFix & getGps() const;

  /**
   * @brief Set a command trajectory point
   *
   * @param trajectory_point trajectory point to publish
   */
  void setCommandTrajectoryPoint(const as2_msgs::msg::TrajectoryPoint & trajectory_point);

  /**
   * @brief Set a command pose
   *
   * @param pose pose to publish
   */
  void setCommandPose(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Set a command twist
   *
   * @param twist twist to publish
   */
  void setCommandTwist(const geometry_msgs::msg::TwistStamped & twist);

  /**
   * @brief Set a command thrust
   *
   * @param thrust thrust to publish
   */
  void setCommandThrust(const as2_msgs::msg::Thrust & thrust);

  /**
   * @brief Send the commands to the drone
  */
  void sendCommands();

  /**
   * @brief Set the Command Send Timer state
   *
   * @param enable true to enable the timer, false to disable
   * @param frequency frequency of the timer (Hz). Default 100 Hz
  */
  void setCommandSendTimerState(const bool enable, const double frequency = 100.0);

  /**
   * @brief Publish an alert event
   *
   * @param alert_event alert event to publish
   */
  void publishAlertEvent(const as2_msgs::msg::AlertEvent & alert_event);

  /**
   * @brief Print the state of the drone
   *
   * @param print_state_period period to print the state (s)
   */
  void printState(const double print_state_period = 1.0);

  /**
   * @brief Wait for a response of a service
   *
   * @tparam ServiceType type of the service
   * @param future future of the service
   * @param spin_executor true to spin the executor, false to wait
   * @return true if the response is success, false otherwise
  */
  template<typename ServiceType>
  bool waitResponse(
    typename rclcpp::Client<ServiceType>::SharedFuture future,
    const bool spin_executor)
  {
    if (spin_executor) {
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    } else {
      future.wait();
    }

    if (future.valid()) {
      auto result = future.get();
      if (result) {
        return result->success;
      }
    }
    return false;
  }

private:
  // Timer
  rclcpp::TimerBase::SharedPtr command_timer_;

  // Services clients
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_srv_cli_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr offboard_srv_cli_;
  rclcpp::Client<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr state_machine_srv_cli_;
  rclcpp::Client<as2_msgs::srv::SetControlMode>::SharedPtr control_mode_srv_cli_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr takeoff_srv_cli_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr land_srv_cli_;

  // Publisher
  rclcpp::Publisher<as2_msgs::msg::TrajectoryPoint>::SharedPtr trajectory_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_pub_;
  rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr thrust_command_pub_;
  rclcpp::Publisher<as2_msgs::msg::AlertEvent>::SharedPtr alert_event_pub_;

  // Subscriber
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ground_truth_twist_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  // Suscribers data
  as2_msgs::msg::PlatformInfo platform_info_;
  geometry_msgs::msg::PoseStamped ground_truth_pose_;
  geometry_msgs::msg::TwistStamped ground_truth_twist_;
  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::NavSatFix gps_;

  // Publishers data
  as2_msgs::msg::TrajectoryPoint trajectory_command_;
  geometry_msgs::msg::PoseStamped pose_command_;
  geometry_msgs::msg::TwistStamped twist_command_;
  as2_msgs::msg::Thrust thrust_command_;
  as2_msgs::msg::AlertEvent alert_event_;

  // Others
  rclcpp::Time last_print_state_time_;

private:
  /**
   * @brief Call a service
   *
   * @tparam ServiceType type of the service
   * @param request request to send
   * @param srv_cli client to call the service
   * @return typename rclcpp::Client<ServiceType>::SharedFuture
   */
  template<typename ServiceType>
  typename rclcpp::Client<ServiceType>::SharedFuture callService(
    typename ServiceType::Request::SharedPtr request,
    typename rclcpp::Client<ServiceType>::SharedPtr srv_cli)
  {
    while (!srv_cli->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the service. Exiting.");
        return typename rclcpp::Client<ServiceType>::SharedFuture();
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Service not available, waiting again...");
    }

    auto result_future = srv_cli->async_send_request(request);
    return result_future;
  }
};

}  // namespace mock
}  // namespace as2

#endif  // AS2_CORE__TESTS__MOCKS__AERIAL_PLATFORM__MOCK_AERIAL_PLATFORM_HPP_
