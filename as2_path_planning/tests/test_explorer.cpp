#include <gtest/gtest.h>

#include <algorithm> // std::sort
#include <iterator>
#include <opencv2/core/types.hpp>
#include <random>
#include <vector>

#include "frontier_utils.hpp"

namespace {

std::vector<cv::Point2i> sortPoints(const std::vector<cv::Point2i> &pts) {
  std::vector<cv::Point2i> pts_cp = pts;
  std::sort(pts_cp.begin(), pts_cp.end(), utils::pt_comp);
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
  auto const max_abs_error = 1 / 1024.f;

  std::vector<cv::Point2d> input = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  EXPECT_TRUE(areDoubleListsEqual(
      input, utils::rotatePoints(input, cv::Point2i(0, 0), 0.0)));

  std::vector<cv::Point2d> out_90 = {{0, 0}, {0, -1}, {0, -2}, {0, -3}};
  EXPECT_TRUE(areDoubleListsEqual(
      out_90, utils::rotatePoints(input, cv::Point2i(0, 0), M_PI / 2)));

  std::vector<cv::Point2d> out_180 = {{0, 0}, {-1, 0}, {-2, 0}, {-3, 0}};
  EXPECT_TRUE(areDoubleListsEqual(
      out_180, utils::rotatePoints(input, cv::Point2i(0, 0), M_PI)));

  std::vector<cv::Point2d> out_270 = {{0, 0}, {0, 1}, {0, 2}, {0, 3}};
  EXPECT_TRUE(areDoubleListsEqual(
      out_270, utils::rotatePoints(input, cv::Point2i(0, 0), 3 * M_PI / 2)));

  std::vector<cv::Point2d> out =
      utils::rotatePoints(input, cv::Point2i(0, 0), M_PI / 4);
  EXPECT_TRUE(areDoubleListsEqual(
      input, utils::rotatePoints(out, cv::Point(0, 0), -M_PI / 4)));
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

TEST(ExplorerTest, FindClosest) {
  std::vector<cv::Point2i> input = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  EXPECT_EQ(input.begin(), utils::closest(input, cv::Point2i(0, 0)));
  EXPECT_EQ(cv::Point2i(1, 0), *utils::closest(input, cv::Point2i(1, 1)));

  std::vector<cv::Point2i> input3 = {
      {0, 0},  {1, 0},  {2, 0},  {3, 0},  {4, 0},  {5, 0},  {6, 0},  {7, 0},
      {8, 0},  {9, 0},  {10, 0}, {11, 0}, {12, 0}, {13, 0}, {14, 0}, {15, 0},
      {16, 0}, {17, 0}, {18, 0}, {19, 0}, {20, 0}, {21, 0}, {22, 0}, {23, 0},
      {24, 0}, {25, 0}, {26, 0}, {27, 0}, {28, 0}, {29, 0}};
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(input3.begin(), input3.end(), g);
  EXPECT_EQ(cv::Point2i(15, 0), *utils::closest(input3, cv::Point2i(15, 15)));
  EXPECT_EQ(cv::Point2i(29, 0), *utils::closest(input3, cv::Point2i(150, 150)));
}

TEST(ExplorerTest, SnakeSort) {
  std::random_device rd;
  std::mt19937 g(rd());

  std::vector<cv::Point2i> input = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  std::vector<cv::Point2i> output = input;
  std::shuffle(input.begin(), input.end(), g);
  EXPECT_EQ(output,
            utils::snakeSort(input, std::find(input.begin(), input.end(),
                                              cv::Point2i(0, 0))));

  std::vector<cv::Point2i> input3 = {
      {0, 0},  {1, 0},  {2, 0},  {3, 0},  {4, 0},  {5, 0},  {6, 0},  {7, 0},
      {8, 0},  {9, 0},  {10, 0}, {11, 0}, {12, 0}, {13, 0}, {14, 0}, {15, 0},
      {16, 0}, {17, 0}, {18, 0}, {19, 0}, {20, 0}, {21, 0}, {22, 0}, {23, 0},
      {24, 0}, {25, 0}, {26, 0}, {27, 0}, {28, 0}, {29, 0}};
  std::vector<cv::Point2i> output3 = input3;
  std::shuffle(input3.begin(), input3.end(), g);
  EXPECT_EQ(output3,
            utils::snakeSort(input3, std::find(input3.begin(), input3.end(),
                                               cv::Point2i(0, 0))));

  std::vector<cv::Point2i> input4 = {{0, 0}, {0, 1}, {0, 2}, {0, 3}, {1, 3},
                                     {2, 3}, {2, 2}, {2, 1}, {2, 0}};
  std::vector<cv::Point2i> output4 = input4;
  std::shuffle(input4.begin(), input4.end(), g);
  EXPECT_EQ(output4,
            utils::snakeSort(input4, std::find(input4.begin(), input4.end(),
                                               cv::Point2i(0, 0))));

  std::vector<cv::Point2i> input5 = {{0, 0}, {1, 1}, {2, 2}, {3, 3},
                                     {4, 4}, {5, 4}, {6, 4}, {6, 3},
                                     {6, 2}, {6, 1}, {6, 0}};
  std::vector<cv::Point2i> output5 = input5;
  std::shuffle(input5.begin(), input5.end(), g);
  EXPECT_EQ(output5,
            utils::snakeSort(input5, std::find(input5.begin(), input5.end(),
                                               cv::Point2i(0, 0))));
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
