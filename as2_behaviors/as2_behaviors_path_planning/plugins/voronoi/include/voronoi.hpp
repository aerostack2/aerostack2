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
 *  \file       voronoi.hpp
 *  \brief      voronoi header file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#ifndef VORONOI_HPP_
#define VORONOI_HPP_

#include <memory>
#include <string>
#include <vector>
#include <as2_behaviors_path_planning/path_planner_plugin_base.hpp>

#include "dynamicvoronoi/dynamicvoronoi.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "cell_node.hpp"
#include "voronoi_searcher.hpp"

namespace voronoi
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
  DynamicVoronoi dynamic_voronoi_;
  unsigned int last_size_x_ = 0;
  unsigned int last_size_y_ = 0;
  std::mutex mutex_;

  VoronoiSearcher graph_searcher_;

  nav_msgs::msg::OccupancyGrid last_occ_grid_;
  nav_msgs::msg::OccupancyGrid last_dist_field_grid_;
  bool enable_visualization_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr viz_voronoi_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr viz_dist_field_grid_pub_;

private:
  void occ_grid_cbk(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  bool outline_map(nav_msgs::msg::OccupancyGrid & occ_grid, uint8_t value);

  void update_dynamic_voronoi(nav_msgs::msg::OccupancyGrid & occ_grid);

  void update_costs(nav_msgs::msg::OccupancyGrid & occ_grid);

  void viz_voronoi_grid();

  void viz_dist_field_grid();

  visualization_msgs::msg::Marker get_path_marker(
    std::string frame_id, rclcpp::Time stamp,
    std::vector<Point2i> path, nav_msgs::msg::MapMetaData map_info,
    std_msgs::msg::Header map_header);
};
}  // namespace voronoi

#endif  // VORONOI_HPP_
