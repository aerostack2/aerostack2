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

/*!******************************************************************************
 *  \file       path_planner_plugin_base.hpp
 *  \brief      path_planner_plugin_base header file.
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernandez-Cortizas
 ********************************************************************************/

#ifndef AS2_BEHAVIORS_PATH_PLANNING__PATH_PLANNER_PLUGIN_BASE_HPP_
#define AS2_BEHAVIORS_PATH_PLANNING__PATH_PLANNER_PLUGIN_BASE_HPP_

#include <memory>
#include <vector>

#include <as2_core/node.hpp>
#include <as2_behavior/behavior_utils.hpp>
#include "as2_msgs/action/navigate_to_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/buffer.h"

namespace as2_behaviors_path_planning
{
class PluginBase
{
public:
  virtual void initialize(as2::Node * node_ptr, std::shared_ptr<tf2_ros::Buffer> tf_buffer) = 0;

  virtual bool on_activate(
    geometry_msgs::msg::PoseStamped drone_pose,
    as2_msgs::action::NavigateToPoint::Goal goal) = 0;
  virtual bool on_deactivate() = 0;
  virtual bool on_modify() = 0;
  virtual bool on_pause() = 0;
  virtual bool on_resume() = 0;
  virtual void on_execution_end() = 0;
  virtual as2_behavior::ExecutionStatus on_run() = 0;

  virtual ~PluginBase() {}

protected:
  PluginBase() {}

protected:
  as2::Node * node_ptr_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

public:
  std::vector<geometry_msgs::msg::Point> path_;
};
}  // namespace as2_behaviors_path_planning

#endif  // AS2_BEHAVIORS_PATH_PLANNING__PATH_PLANNER_PLUGIN_BASE_HPP_
