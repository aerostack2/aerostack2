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
 *  \file       mpc.hpp
 *  \brief      Declares the controller plugin Model Predictive Controller (MPC) for the Aerostack framework.
 *  \authors    Rafael Pérez Seguí
 ********************************************************************************************/

#ifndef MPC__MPC_HPP_
#define MPC__MPC_HPP_

#include <vector>
#include <string>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"

#include "as2_motion_controller/controller_base.hpp"

#include "acados_mpc/acados_mpc.hpp"

namespace mpc
{

struct Control_flags
{
  bool parameters_read = false;
  bool ref_received = false;
  bool hover_flag_ = false;
};

class Plugin : public as2_motion_controller_plugin_base::ControllerBase
{
public:
  Plugin() {}
  ~Plugin() {}

  /** Virtual functions from ControllerBase */
  void ownInitialize() override;

  void updateState(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const geometry_msgs::msg::TwistStamped & twist_msg) override;

  void updateReference(const as2_msgs::msg::TrajectorySetpoints & ref) override;

  bool computeOutput(
    double dt,
    geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::TwistStamped & twist,
    as2_msgs::msg::Thrust & thrust) override;

  bool setMode(
    const as2_msgs::msg::ControlMode & mode_in,
    const as2_msgs::msg::ControlMode & mode_out) override;

  bool updateParams(const std::vector<rclcpp::Parameter> & _params_list) override;

  void reset() override;

  // IMPORTANT: this is the frame_id of the desired pose and twist,
  // both reference and state
  std::string getDesiredPoseFrameId() override {return odom_frame_id_;}
  std::string getDesiredTwistFrameId() override {return odom_frame_id_;}

private:
  bool checkParamList(const std::string & param, std::vector<std::string> & _params_list);

private:
  // Internal variables
  Control_flags flags_;
  as2_msgs::msg::ControlMode control_mode_in_;

  const std::vector<std::string> parameters_list_ = {
    "mpc.Q",
    "mpc.Qe",
    "mpc.R",
    "mpc.lbu",
    "mpc.ubu",
    "mpc.p"
  };
  std::vector<std::string> parameters_to_read_{parameters_list_};


  // MPC
  acados_mpc::MPC mpc_;
  acados_mpc::MPCData * mpc_data_ = nullptr;

  // TODO(RPS98): Remove this
  std::string odom_frame_id_ = "odom";
  std::string base_link_frame_id_ = "odom";
};  // class Plugin
}   // namespace mpc

#endif  // MPC__MPC_HPP_
