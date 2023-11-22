#ifndef FRONTIER_ALLOCATOR_HPP_
#define FRONTIER_ALLOCATOR_HPP_

#include <as2_msgs/srv/allocate_frontier.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "frontier_utils.hpp"
#include "utils.hpp"
#include "viz_utils.hpp"

struct Frontier {
  // std::vector<geometry_msgs::msg::PointStamped> points;
  geometry_msgs::msg::PointStamped goal;     // [m]
  geometry_msgs::msg::PointStamped centroid; // [m]
  double area;                               // perimeter [n px]
  double orientation;                        // main vector or orientation [rad]
  cv::Mat labeled_mat; // pixels of the frontier are non zero

  bool operator==(const Frontier &f) {
    // Close, not exactly equal
    return close(f.centroid) && abs(area - f.area) < 5.0 &&
           abs(orientation - f.orientation) < 0.1;
  }

  bool close(const geometry_msgs::msg::PointStamped &centr) {
    return abs(centr.point.x - centroid.point.x) < 0.5 &&
           abs(centr.point.y - centroid.point.y) < 0.5;
  }
};

class FrontierAllocator : public rclcpp::Node {
public:
  FrontierAllocator();
  ~FrontierAllocator(){};

private:
  double safety_distance_ = 0.3; // [m]
  int frontier_min_area_ = 15;   // in pixels
  int frontier_max_area_ = 25;   // in pixels

  std::map<std::string, Frontier> allocated_frontiers_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_occ_grid_ = nullptr;

  void occGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void allocateFrontierCbk(
      const as2_msgs::srv::AllocateFrontier::Request::SharedPtr request,
      as2_msgs::srv::AllocateFrontier::Response::SharedPtr response);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Service<as2_msgs::srv::AllocateFrontier>::SharedPtr
      allocate_frontier_srv_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;

protected:
  void getFrontiers(const nav_msgs::msg::OccupancyGrid &occ_grid,
                    std::vector<Frontier> &frontiersOutput);

  void splitFrontier(const cv::Mat &frontier, int n_parts,
                     std::vector<Frontier> &frontiersOutput);

  void splitFrontierSnake(const cv::Mat &frontier, int n_parts,
                          std::vector<Frontier> &frontiersOutput);
  void visualizeFrontiers(const std::vector<Frontier> &frontiers);

  Frontier explorationHeuristic(const geometry_msgs::msg::PointStamped &goal,
                                const std::vector<Frontier> &frontiers);
  Frontier explorationHeuristic(const geometry_msgs::msg::PoseStamped &goal,
                                const std::vector<Frontier> &frontiers);

  Frontier getCloserFrontier(const geometry_msgs::msg::PointStamped &goal,
                             const std::vector<Frontier> &frontiers);
};
#endif // FRONTIER_ALLOCATOR_HPP_