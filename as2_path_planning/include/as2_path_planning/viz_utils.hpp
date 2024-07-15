#ifndef VIZ_UTILS_HPP_
#define VIZ_UTILS_HPP_

#include <nav_msgs/msg/map_meta_data.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "utils.hpp"

namespace utils {

inline void cleanMarkers(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub,
    std::string ns, int id = -1) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "earth";
  marker.header.stamp = rclcpp::Time();
  marker.ns = ns;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  viz_pub->publish(marker);
}

inline visualization_msgs::msg::Marker
getPointMarker(std::string name, int id, std_msgs::msg::Header header,
               geometry_msgs::msg::Point point) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = name;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.scale.x = 0.25; // Set the scale of the marker
  marker.scale.y = 0.25; // Set the scale of the marker
  marker.scale.z = 0.25; // Set the scale of the marker
  marker.lifetime =
      rclcpp::Duration::from_seconds(0); // Set the lifetime of the marker

  marker.pose.position = point;
  return marker;
}

inline visualization_msgs::msg::Marker
getTextMarker(std::string name, int id, std_msgs::msg::Header header,
              geometry_msgs::msg::Point point, std::string text) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = name;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.scale.x = 0.25; // Set the scale of the marker
  marker.scale.y = 0.25; // Set the scale of the marker
  marker.scale.z = 0.25; // Set the scale of the marker
  marker.lifetime =
      rclcpp::Duration::from_seconds(0); // Set the lifetime of the marker

  marker.pose.position = point;
  marker.pose.position.z += 0.5;
  marker.text = text;
  return marker;
}

inline visualization_msgs::msg::Marker
getPointMarker(std::string name, int id, std_msgs::msg::Header header,
               cv::Point2i pixel, nav_msgs::msg::MapMetaData map_info,
               std_msgs::msg::Header map_header) {

  auto point = utils::pixelToPoint(pixel, map_info, map_header);
  return getPointMarker(name, id, header, point.point);
}

inline visualization_msgs::msg::Marker
getPathMarker(std::string frame_id, rclcpp::Time stamp,
              std::vector<cv::Point> path, nav_msgs::msg::MapMetaData map_info,
              std_msgs::msg::Header map_header) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "map_server";
  marker.id = 33;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.lifetime = rclcpp::Duration::from_seconds(0); // Lifetime forever

  for (auto &p : path) {
    auto point = utils::pixelToPoint(p, map_info, map_header);
    marker.points.push_back(point.point);
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    marker.colors.push_back(color);
  }
  return marker;
}

inline visualization_msgs::msg::Marker
getFrontierMarker(int id, std::vector<cv::Point> path,
                  nav_msgs::msg::MapMetaData map_info,
                  std_msgs::msg::Header map_header) {
  visualization_msgs::msg::Marker marker;
  marker.header = map_header;
  marker.ns = "frontier";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.lifetime = rclcpp::Duration::from_seconds(0); // Lifetime forever

  double R = ((double)rand() / (double)RAND_MAX);
  double G = ((double)rand() / (double)RAND_MAX);
  double B = ((double)rand() / (double)RAND_MAX);

  for (auto &p : path) {
    auto point = utils::pixelToPoint(p.y, p.x, map_info, map_header);
    marker.points.push_back(point.point);
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.r = R;
    color.g = G;
    color.b = B;
    marker.colors.push_back(color);
  }
  return marker;
}

} // namespace utils

#endif // VIZ_UTILS_HPP_