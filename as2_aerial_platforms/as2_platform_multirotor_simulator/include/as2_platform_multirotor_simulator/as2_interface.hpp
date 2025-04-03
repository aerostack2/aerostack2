// Copyright 2025 Universidad Politécnica de Madrid
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
* @file as2_interface.hpp
*
* As2MultirotorSimulatorInterface class definition
*
* @authors Rafael Pérez Seguí
*/

#ifndef AS2_PLATFORM_MULTIROTOR_SIMULATOR__AS2_INTERFACE_HPP_
#define AS2_PLATFORM_MULTIROTOR_SIMULATOR__AS2_INTERFACE_HPP_

#include <string>
#include <vector>

#include "multirotor_simulator.hpp"
#include "as2_core/aerial_platform.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <as2_msgs/msg/trajectory_setpoints.hpp>
#include <as2_msgs/msg/trajectory_point.hpp>

namespace as2_platform_multirotor_simulator
{

class As2MultirotorSimulatorInterface
{
  using Simulator = multirotor::Simulator<double, 4>;
  using SimulatorParams = multirotor::SimulatorParams<double, 4>;
  using Kinematics = multirotor::state::internal::Kinematics<double>;

public:
  explicit As2MultirotorSimulatorInterface(
    as2::Node * node_ptr);

  ~As2MultirotorSimulatorInterface() {}

private:
  as2::Node * node_ptr_;

  // Tf
  as2::tf::TfHandler tf_handler_;
  std::string frame_id_baselink_ = "base_link";
  std::string frame_id_odom_ = "odom";
  std::string frame_id_earth_ = "earth";

  Eigen::Vector3d initial_position_;
  Eigen::Quaterniond initial_orientation_;
  bool using_odom_for_control_ = false;

public:
  /**
    * @brief Get parameter from the parameter server
    *
    * @param param_name Name of the parameter
    * @param param_value Value of the parameter
    * @param use_default Use default value if parameter is not found
   */
  template<typename T>
  inline void getParam(const std::string & param_name, T & param_value, bool use_default = false)
  {
    try {
      // Declare parameter if not declared
      if (!node_ptr_->has_parameter(param_name)) {
        if (use_default) {
          node_ptr_->declare_parameter<T>(param_name, param_value);
        } else {
          node_ptr_->declare_parameter<T>(param_name);
        }
      }

      if constexpr (std::is_same<T, std::vector<double>>::value) {
        param_value = node_ptr_->get_parameter(param_name).as_double_array();
      } else if constexpr (std::is_same<T, double>::value) {
        param_value = node_ptr_->get_parameter(param_name).as_double();
      } else if constexpr (std::is_same<T, std::string>::value) {
        param_value = node_ptr_->get_parameter(param_name).as_string();
      } else if constexpr (std::is_same<T, bool>::value) {
        param_value = node_ptr_->get_parameter(param_name).as_bool();
      } else {
        RCLCPP_WARN(node_ptr_->get_logger(), "Parameter type %s not expected", typeid(T).name());
        param_value = node_ptr_->get_parameter<T>(param_name, param_value);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Error getting parameter %s: %s", param_name.c_str(), e.what());
    }
  }

  /**
   * @brief Convert simulator data to odometry message
   *
   * @param kinematics Kinematics data
   * @param odom Return odometry message
   * @param current_time Current time
   */
  void convertToOdom(
    const Kinematics & kinematics, nav_msgs::msg::Odometry & odom,
    const builtin_interfaces::msg::Time & current_time);

  /**
   * @brief Convert simulator data to ground truth message
   *
   * @param kinematics Kinematics data
   * @param ground_truth_pose Return ground truth pose message
   * @param ground_truth_twist Return ground truth twist message
   * @param current_time Current time
   */
  void convertToGroundTruth(
    const Kinematics & kinematics, geometry_msgs::msg::PoseStamped & ground_truth_pose,
    geometry_msgs::msg::TwistStamped & ground_truth_twist,
    const builtin_interfaces::msg::Time & current_time);

  bool processCommand(
    geometry_msgs::msg::PoseStamped & pose_command);

  bool processCommand(
    geometry_msgs::msg::TwistStamped & twist_command);

  bool processCommand(as2_msgs::msg::TrajectorySetpoints trajectory_command);
};  // class As2MultirotorSimulatorInterface
}  // namespace as2_platform_multirotor_simulator

#endif  // AS2_PLATFORM_MULTIROTOR_SIMULATOR__AS2_INTERFACE_HPP_
