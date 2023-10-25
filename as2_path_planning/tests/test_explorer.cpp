#include <gtest/gtest.h>

#include <algorithm> // std::sort
#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "explorer.hpp"

namespace {

bool pt_comp(cv::Point2i pt1, cv::Point2i pt2) {
  if (pt1.x == pt2.x) {
    return pt1.y < pt2.y;
  }
  return pt1.x < pt2.x;
}

std::vector<cv::Point2i> sortPoints(const std::vector<cv::Point2i> &pts) {
  std::vector<cv::Point2i> pts_cp = pts;
  std::sort(pts_cp.begin(), pts_cp.end(), pt_comp);
  return pts_cp;
}

bool areDoubleListsEqual(const std::vector<cv::Point2d> &l1,
                         const std::vector<cv::Point2d> &l2) {
  if (l1.size() != l2.size()) {
    return false;
  }

  for (size_t i = 0; i < l1.size(); ++i) {
    if (std::abs(l1[i].x - l2[i].x) > 1e-8) {
      return false;
    }
    if (std::abs(l1[i].y - l2[i].y) > 1e-8) {
      return false;
    }
  }
  return true;
}

TEST(ExplorerTest, Rotation) {
  rclcpp::init(0, nullptr);

  auto const max_abs_error = 1 / 1024.f;

  auto explorer = std::make_shared<Explorer>();
  std::vector<cv::Point2d> input = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  EXPECT_TRUE(areDoubleListsEqual(
      input, explorer->rotatePoints(input, cv::Point2i(0, 0), 0.0)));

  std::vector<cv::Point2d> out_90 = {{0, 0}, {0, -1}, {0, -2}, {0, -3}};
  EXPECT_TRUE(areDoubleListsEqual(
      out_90, explorer->rotatePoints(input, cv::Point2i(0, 0), M_PI / 2)));

  std::vector<cv::Point2d> out_180 = {{0, 0}, {-1, 0}, {-2, 0}, {-3, 0}};
  EXPECT_TRUE(areDoubleListsEqual(
      out_180, explorer->rotatePoints(input, cv::Point2i(0, 0), M_PI)));

  std::vector<cv::Point2d> out_270 = {{0, 0}, {0, 1}, {0, 2}, {0, 3}};
  EXPECT_TRUE(areDoubleListsEqual(
      out_270, explorer->rotatePoints(input, cv::Point2i(0, 0), 3 * M_PI / 2)));

  std::vector<cv::Point2d> out =
      explorer->rotatePoints(input, cv::Point2i(0, 0), M_PI / 4);
  EXPECT_TRUE(areDoubleListsEqual(
      input, explorer->rotatePoints(out, cv::Point(0, 0), -M_PI / 4)));

  rclcpp::shutdown();
}

TEST(ExplorerTest, Sorting) {
  std::vector<cv::Point2i> input = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  EXPECT_EQ(input, sortPoints(input));

  std::vector<cv::Point2i> input2 = {{0, 0}, {3, 0}, {2, 0}, {1, 0}};
  EXPECT_EQ(input, sortPoints(input));

  std::vector<cv::Point2i> input3 = {{0, 0}, {0, 1}, {0, 2}, {0, 3}};
  EXPECT_EQ(input3, sortPoints(input3));

  std::vector<cv::Point2i> input4 = {{0, 0}, {0, 3}, {0, 2}, {0, 1}};
  EXPECT_EQ(input3, sortPoints(input4));
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
