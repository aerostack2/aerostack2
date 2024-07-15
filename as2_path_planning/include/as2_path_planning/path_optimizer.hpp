#ifndef PATH_OPTIMIZER_HPP_
#define PATH_OPTIMIZER_HPP_

#include <opencv2/core/types.hpp>
#include <vector>

namespace path_optimizer {

inline float slope(cv::Point2i p1, cv::Point2i p2) {
  if (p1.x == p2.x) {
    return 0.0;
  }
  return (float)(p2.y - p1.y) / (float)(p2.x - p1.x);
}

/* Expected input should not contain repeated consecutive points. */
inline std::vector<cv::Point2i> solve(const std::vector<cv::Point2i> &input) {
  std::vector<cv::Point2i> output;

  // Handle the case when there are not enough points to process.
  if (input.size() < 3) {
    return input;
  }

  cv::Point2i p1 = input[0];
  cv::Point2i p2 = input[1];
  // y = mx + n
  float m = slope(p1, p2);
  output.push_back(p1);

  for (size_t i = 2; i < input.size(); ++i) {
    cv::Point2i p = input[i];

    if (m != slope(p1, p)) {
      output.push_back(input[i - 1]);
      p1 = input[i - 1];
      p2 = p;
      m = slope(p1, p2);
    }
  }

  output.push_back(input.back());
  return output;
}

} // namespace path_optimizer

#endif // PATH_OPTIMIZER_HPP_