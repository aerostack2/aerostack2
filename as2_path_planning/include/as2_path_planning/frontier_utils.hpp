#ifndef FRONTIER_UTILS_HPP_
#define FRONTIER_UTILS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace utils {

// L2 distance between two 2d points
inline double distance(geometry_msgs::msg::Point p1,
                       geometry_msgs::msg::Point p2) {
  double dis = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
  return dis;
}

// Comparison between two cv::Point
inline bool pt_comp(cv::Point pt1, cv::Point pt2) {
  if (pt1.x == pt2.x) {
    return pt1.y < pt2.y;
  }
  return pt1.x < pt2.x;
}

// Rotate a vector of points around an origin
inline std::vector<cv::Point2d>
rotatePoints(const std::vector<cv::Point2d> &pts, const cv::Point2d &origin,
             double angle) {
  std::vector<cv::Point2d> rotated_pts;
  for (const cv::Point2d &pt : pts) {
    double x = (pt.x - origin.x) * std::cos(angle) +
               (pt.y - origin.y) * std::sin(angle) + origin.x;
    double y = -(pt.x - origin.x) * std::sin(angle) +
               (pt.y - origin.y) * std::cos(angle) + origin.y;
    rotated_pts.push_back(cv::Point2d(x, y));
  }
  return rotated_pts;
}

/* Find closest point in vector to pt*/
inline std::vector<cv::Point2i>::const_iterator
closest(const std::vector<cv::Point2i> &vec, cv::Point2i pt) {
  double minDist = std::numeric_limits<double>::max();
  std::vector<cv::Point2i>::const_iterator closest;
  for (std::vector<cv::Point2i>::const_iterator it = vec.begin();
       it != vec.end(); ++it) {
    cv::Point2i diff = *it - pt;
    double dist = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
    // double dist = std::abs(diff.x) + std::abs(diff.y);
    if (dist < minDist) {
      minDist = dist;
      closest = it;
    }
  }

  return closest;
}

inline std::vector<cv::Point2i>::const_iterator
closest(const std::vector<cv::Point2i> &vec, int pt_x, int pt_y) {
  return closest(vec, cv::Point2i(pt_x, pt_y));
}

/*
Sorting points from closest to start to furthest
Use carefully, this function modifies the input vector
*/
inline std::vector<cv::Point2i>
snakeSort(std::vector<cv::Point2i> &pts,
          const std::vector<cv::Point2i>::iterator &start) {
  std::vector<cv::Point2i> sorted_pts;
  sorted_pts.push_back(*start);
  pts.erase(start);

  cv::Point2i current = sorted_pts[0];
  while (pts.size() > 0) {
    auto neightbor = utils::closest(pts, current);
    current = *neightbor;
    sorted_pts.push_back(*neightbor);
    pts.erase(neightbor);
  }
  return sorted_pts;
}

} // namespace utils

#endif // FRONTIER_UTILS_HPP_