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

/**
* @file as2_geozones.hpp
*
* as2_geozones header file.
*
* @author Javilinos
*/

#ifndef AS2_GEOZONES__AS2_GEOZONES_HPP_
#define AS2_GEOZONES__AS2_GEOZONES_HPP_

#include <limits>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "as2_msgs/msg/geozone.hpp"
#include "as2_msgs/msg/polygon_list.hpp"
#include "as2_msgs/srv/get_geozone.hpp"
#include "as2_msgs/srv/set_geozone.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/utils/gps_utils.hpp"
#include "as2_msgs/msg/alert_event.hpp"
#include "as2_msgs/srv/get_origin.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/int8.hpp"

#include "pnpoly.hpp"

class Geozones : public as2::Node
{
public:
  Geozones();

  void setupNode();
  void cleanupNode();
  void run();
  void loadGeozones(const std::string path);

private:
  struct geozone  // Structure declaration
  {
    std::string type;  // geofence or geocage
    std::vector<std::array<double, 2>>
    polygon;     // polygons that define each geozone !!order matters!!
    float z_up;
    float z_down;
    int id;                // id of geozone
    int alert;             // alert that generates
    std::string data_type;  // type of geozone: cartesian or gps
    bool in;               // if point is in the geofence
  };

  bool start_run_;
  bool origin_set_ = false;

  float self_latitude_;
  float self_longitude_;
  float self_altitude_;
  float self_x_;
  float self_y_;
  float self_z_;
  int max_priority;
  bool geofence_detected;

  std::string config_path_;
  bool rviz_visualization_ = false;
  std::array<double, 2> point_;
  std::vector<geozone> geozones_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<as2_msgs::msg::AlertEvent>::SharedPtr alert_pub_;
  rclcpp::Publisher<as2_msgs::msg::PolygonList>::SharedPtr rviz_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<as2_msgs::srv::SetGeozone>::SharedPtr set_geozone_srv_;
  rclcpp::Service<as2_msgs::srv::GetGeozone>::SharedPtr get_geozone_srv_;

  geographic_msgs::msg::GeoPoint::UniquePtr origin_;
  rclcpp::Client<as2_msgs::srv::GetOrigin>::SharedPtr get_origin_srv_;
  std::unique_ptr<as2::gps::GpsHandler> gps_handler;

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr _msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
  void setGeozoneCb(
    const std::shared_ptr<as2_msgs::srv::SetGeozone::Request> request,
    std::shared_ptr<as2_msgs::srv::SetGeozone::Response> response);
  void getGeozoneCb(
    const std::shared_ptr<as2_msgs::srv::GetGeozone::Request> request,
    std::shared_ptr<as2_msgs::srv::GetGeozone::Response> response);

  void rvizVisualizationCb();

  void checkGeozones();
  bool checkValidity(int size, int id, std::string type, std::string data_type);
  bool findGeozoneId(int id);
  void setupGPS();

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
};

#endif  // AS2_GEOZONES__AS2_GEOZONES_HPP_
