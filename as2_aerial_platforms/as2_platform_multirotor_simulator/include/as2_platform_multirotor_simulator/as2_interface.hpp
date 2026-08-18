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

#include <cassert>
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
  /**
   * @brief Construct the simulator interface, reading the TF frame names and
   * the use_odom_for_control flag from the node parameters.
   *
   * @param node_ptr Platform node the parameters are read from.
   */
  explicit As2MultirotorSimulatorInterface(
    as2::Node * node_ptr);

  /**
   * @brief Destroy the As2 Multirotor Simulator Interface object.
   */
  ~As2MultirotorSimulatorInterface() {}

private:
  as2::Node * node_ptr_;

  // Tf
  as2::tf::TfHandler tf_handler_;
  std::string frame_id_baselink_;
  std::string frame_id_odom_;
  std::string frame_id_earth_;

  Eigen::Vector3d initial_position_;
  Eigen::Quaterniond initial_orientation_;
  bool using_odom_for_control_ = false;

public:
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

  /**
   * @brief Convert a pose command to the frame the simulator is controlled in.
   *
   * @param pose_command Pose command, converted in place.
   * @return true if the command is expressed in the frame of the simulator.
   */
  bool processCommand(
    geometry_msgs::msg::PoseStamped & pose_command);

  /**
   * @brief Convert a twist command to the frame the simulator is controlled in.
   *
   * @param twist_command Twist command, converted in place.
   * @return true if the command is expressed in the frame of the simulator.
   */
  bool processCommand(
    geometry_msgs::msg::TwistStamped & twist_command);

  /**
   * @brief Convert a trajectory command to the frame the simulator is controlled in.
   *
   * @param trajectory_command Trajectory command, taken by value.
   * @return true if the command is expressed in the frame of the simulator.
   */
  bool processCommand(as2_msgs::msg::TrajectorySetpoints trajectory_command);
};  // class As2MultirotorSimulatorInterface
}  // namespace as2_platform_multirotor_simulator

#endif  // AS2_PLATFORM_MULTIROTOR_SIMULATOR__AS2_INTERFACE_HPP_
