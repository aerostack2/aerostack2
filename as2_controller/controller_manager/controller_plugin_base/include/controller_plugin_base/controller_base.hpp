/********************************************************************************************
 *  \file       controller_base.hpp
 *  \brief      Declares the controller_plugin_base class which is the base
 *class for all controller plugins.
 *
 *  \authors    Miguel Fernández Cortizas
 *              Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *              David Pérez Saura
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

#ifndef CONTROLLER_BASE_HPP
#define CONTROLLER_BASE_HPP


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_point.hpp"

namespace controller_plugin_base {

class ControllerBase {
public:
  /*
   * @brief Constructor
   */
  ControllerBase(){};

  /*
   * @brief Initialize the controller plugin, since it is a plugin the Constructor must not have
   * parameters
   * @param node_ptr as2::Node pointer to be used by the controller plugin
   */
  void initialize(as2::Node* node_ptr) {
    node_ptr_ = node_ptr;
    ownInitialize();
  };

  /*
   * @brief Own initialize function to be implemented by the controller plugin
   */
  virtual void ownInitialize(){};

  /*
   * @brief Update the State obtained from the sensors to be used by the controller plugin
   * @param pose_msg geometry_msgs::msg::PoseStamped message with the current pose of the robot in
   * the "odom" frame
   * @param twist_msg geometry_msgs::msg::TwistStamped message with the current twist of the robot
   * in the "base_link" frame
   */
  virtual void updateState(const geometry_msgs::msg::PoseStamped& pose_msg,
                           const geometry_msgs::msg::TwistStamped& twist_msg) = 0;

  /*
   * @brief Update the pose reference to be used by the controller plugin
   * @param ref geometry_msgs::msg::PoseStamped message with the current pose of the robot in
   * the "odom" frame
   */
  virtual void updateReference(const geometry_msgs::msg::PoseStamped& ref){};

  /*
   * @brief Update the speedreference to be used by the controller plugin
   * @param ref geometry_msgs::msg::TwistStamped message with the current twist of the robot in the
   * "base_link" frame
   */
  virtual void updateReference(const geometry_msgs::msg::TwistStamped& ref){};
  /*
   * @brief Update the reference to be used by the controller plugin
   * @param ref as2_msgs::msg::TrajectoryPoint message with the current reference of
   * the robot in the "odom" frame
   */
  virtual void updateReference(const as2_msgs::msg::TrajectoryPoint& ref){};

  // virtual void updateReference(const as2_msgs::msg::Thrust& ref){};

  /*
   * @brief Compute the output signal of the controller plugin
   * @param pose geometry_msgs::msg::PoseStamped message with the output pose of the robot. The
   * frame will depend on the output control mode
   * @param twist geometry_msgs::msg::TwistStamped message with the output twist of the robot. The
   * frame will depend on the output control mode
   * @param thrust as2_msgs::msg::Thrust message with the output thrust of the robot
   */
  virtual bool computeOutput(double dt,
                             geometry_msgs::msg::PoseStamped& pose,
                             geometry_msgs::msg::TwistStamped& twist,
                             as2_msgs::msg::Thrust& thrust) = 0;
  /*
   * @brief Update the control mode to be used by the controller plugin
   * @param mode_in as2_msgs::msg::ControlMode message with the desired input control mode
   * @param mode_out as2_msgs::msg::ControlMode message with the desired output control mode
   * @return bool true if the in-out control mode configuration is valid, false otherwise
   */
  virtual bool setMode(const as2_msgs::msg::ControlMode& mode_in,
                       const as2_msgs::msg::ControlMode& mode_out) = 0;

  /*
   * @brief Update the parameters of the controller plugin
   * @param params std::vector<std::string> vector with the parameters of the Controller Manager
   * Node
   * @return bool true if the parameters are updated correctly, false otherwise
   */
  virtual bool updateParams(const std::vector<std::string>& _params_list) = 0;

  /*
   * @brief Reset the internal state of the controller plugin
   */
  virtual void reset() = 0;

  /*
   * @brief Get the desired frame_id of the state and reference pose msgs
   * By default it is "odom"
   */
  virtual std::string getDesiredPoseFrameId() { return "odom"; }

  /*
   * @brief Get the desired frame_id of the state and reference twist msgs
   * By default it is "base_link"
   */
  virtual std::string getDesiredTwistFrameId() { return "base_link"; }

  /*
   * @brief Destructor
   */
  virtual ~ControllerBase(){};

protected:
  /* @brief as2::Node pointer to be used by the controller plugin */
  inline as2::Node* getNodePtr() { return node_ptr_; }
  as2::Node* node_ptr_;

};  //  ControllerBase

};  // namespace controller_plugin_base

#endif  // CONTROLLER_BASE_HPP
