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
 *  \file       mapping_2d.hpp
 *  \brief      2d mapping plugin.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#ifndef MAPPING_2D_HPP_
#define MAPPING_2D_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <vector>
#include <as2_map_server/plugin_base.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

namespace mapping_2d
{

class Plugin : public as2_map_server_plugin_base::MapServerBase
{
public:
  Plugin()
  : as2_map_server_plugin_base::MapServerBase() {}

  void on_setup() override;

private:
  double scan_range_max_;  // [m]
  double map_resolution_;  // [m/cell]
  int map_width_;  // [cells]
  int map_height_;  // [cells]

  nav_msgs::msg::OccupancyGrid::SharedPtr occ_grid_ =
    std::make_shared<nav_msgs::msg::OccupancyGrid>();

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_filtered_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

private:
  void on_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void publish_map(const nav_msgs::msg::OccupancyGrid & map_update);

  // AUX METHODS
  std::vector<std::vector<int>> get_middle_points(
    std::vector<int> p1,
    std::vector<int> p2);
  bool is_cell_index_valid(std::vector<int> cell);

  std::vector<int8_t> add_occ_grid_update(
    const std::vector<int8_t> & update, const std::vector<int8_t> & occ_grid_data);
  nav_msgs::msg::OccupancyGrid filter_occ_grid(const nav_msgs::msg::OccupancyGrid & occ_grid);

  /* Point to occupancy grid cell */
  // TODO(parias): change from vector to pair or array
  std::vector<int> point_to_cell(
    geometry_msgs::msg::PointStamped point, nav_msgs::msg::MapMetaData map_info,
    std::string target_frame_id, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

/*
 * Occupancy grid to binary image
 *
 * @param occ_grid: occupancy grid
 * @param thresh: threshold value
 * @return: binary image
 */
  cv::Mat grid_to_img(
    nav_msgs::msg::OccupancyGrid occ_grid,
    double thresh = 30, bool unknown_as_free = false);

/*
 * Binary image to occupancy grid
 *
 * @param img: binary image
 * @param header: header of the occupancy grid
 * @param grid_resolution: resolution of the occupancy grid
 * @return: occupancy grid
 */
  nav_msgs::msg::OccupancyGrid img_to_grid(
    const cv::Mat img, const std_msgs::msg::Header & header,
    double grid_resolution);
};

}  // namespace mapping_2d

#endif  // MAPPING_2D_HPP_
