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
 *  \file       a_star.hpp
 *  \brief      a_star header file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#ifndef A_STAR_HPP_
#define A_STAR_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <as2_behaviors_path_planning/path_planner_plugin_base.hpp>

#include "a_star_algorithm.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include <opencv2/core/types.hpp>
#include "builtin_interfaces/msg/duration.hpp"


namespace a_star
{
class Plugin : public as2_behaviors_path_planning::PluginBase
{
public:
  void initialize(as2::Node * node_ptr, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;

  bool on_activate(
    geometry_msgs::msg::PoseStamped drone_pose,
    as2_msgs::action::NavigateToPoint::Goal goal) override;
  bool on_deactivate() override;
  bool on_modify() override;
  bool on_pause() override;
  bool on_resume() override;
  void on_execution_end() override;
  as2_behavior::ExecutionStatus on_run() override;

private:
  // TODO(pariaspe): Better integration. Should be one file only
  AStarPlanner planner_algorithm_;
  nav_msgs::msg::OccupancyGrid last_occ_grid_;
  double safety_distance_;  // [m]
  bool use_path_optimizer_;
  bool enable_visualization_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr viz_obstacle_grid_pub_;

private:
  void occ_grid_cbk(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  visualization_msgs::msg::Marker get_path_marker(
    std::string frame_id, rclcpp::Time stamp,
    std::vector<cv::Point> path, nav_msgs::msg::MapMetaData map_info,
    std_msgs::msg::Header map_header);
};
}  // namespace a_star

#endif  // A_STAR_HPP_
