#include <gtest/gtest.h>

#include <opencv2/core/types.hpp>
#include <vector>

#include "path_optimizer.hpp"

namespace {

TEST(PathOptimizer, ShortLength) {
  std::vector<cv::Point2i> input = {};
  EXPECT_EQ(path_optimizer::solve(input), input);

  input.push_back({0, 0});
  EXPECT_EQ(path_optimizer::solve(input), input);

  input.push_back({10, 0});
  EXPECT_EQ(path_optimizer::solve(input), input);
}

TEST(PathOptimizer, Line) {
  std::vector<cv::Point2i> input = {};
  input.push_back({0, 0});
  input.push_back({1, 2});
  input.push_back({2, 4});
  input.push_back({3, 6});
  input.push_back({4, 8});

  std::vector<cv::Point2i> expected_output = {};
  expected_output.push_back({0, 0});
  expected_output.push_back({4, 8});
  EXPECT_EQ(path_optimizer::solve(input), expected_output);
}

TEST(PathOptimizer, VLine) {
  std::vector<cv::Point2i> input = {};
  input.push_back({2, 0});
  input.push_back({2, 2});
  input.push_back({2, 4});
  input.push_back({2, 6});
  input.push_back({2, 8});

  std::vector<cv::Point2i> expected_output = {};
  expected_output.push_back({2, 0});
  expected_output.push_back({2, 8});
  EXPECT_EQ(path_optimizer::solve(input), expected_output);
}

TEST(PathOptimizer, HLine) {
  std::vector<cv::Point2i> input = {};
  input.push_back({0, 2});
  input.push_back({2, 2});
  input.push_back({4, 2});
  input.push_back({6, 2});
  input.push_back({8, 2});

  std::vector<cv::Point2i> expected_output = {};
  expected_output.push_back({0, 2});
  expected_output.push_back({8, 2});
  EXPECT_EQ(path_optimizer::solve(input), expected_output);
}

TEST(PathOptimizer, TwoSegmentLine) {
  std::vector<cv::Point2i> input = {};
  input.push_back({0, 0});
  input.push_back({1, 2});
  input.push_back({2, 4});
  input.push_back({4, 5});
  input.push_back({6, 6});

  std::vector<cv::Point2i> expected_output = {};
  expected_output.push_back({0, 0});
  expected_output.push_back({2, 4});
  expected_output.push_back({6, 6});
  EXPECT_EQ(path_optimizer::solve(input), expected_output);
}

TEST(PathOptimizer, ThreeSegmentLine) {
  std::vector<cv::Point2i> input = {};
  input.push_back({0, 0});
  input.push_back({1, 2});
  input.push_back({2, 4});
  input.push_back({6, 6});
  input.push_back({7, 7});
  input.push_back({8, 8});

  std::vector<cv::Point2i> expected_output = {};
  expected_output.push_back({0, 0});
  expected_output.push_back({2, 4});
  expected_output.push_back({6, 6});
  expected_output.push_back({8, 8});

  EXPECT_EQ(path_optimizer::solve(input), expected_output);
}

TEST(PathOptimizer, PolyLine) {
  std::vector<cv::Point2i> input = {};
  input.push_back({0, 0});
  input.push_back({1, 1});
  input.push_back({2, 0});
  input.push_back({3, 1});
  input.push_back({4, 0});
  input.push_back({5, 1});
  input.push_back({6, 0});
  input.push_back({7, 1});

  EXPECT_EQ(path_optimizer::solve(input), input);
}

TEST(PathOptimizer, Square) {
  std::vector<cv::Point2i> input = {};
  input.push_back({0, 0});
  input.push_back({0, 1});
  input.push_back({0, 2});
  input.push_back({1, 2});
  input.push_back({2, 2});
  input.push_back({2, 1});
  input.push_back({2, 0});
  input.push_back({1, 0});
  input.push_back({0, 0});

  std::vector<cv::Point2i> expected_output = {};
  expected_output.push_back({0, 0});
  expected_output.push_back({0, 2});
  expected_output.push_back({2, 2});
  expected_output.push_back({2, 0});
  expected_output.push_back({0, 0});
  EXPECT_EQ(path_optimizer::solve(input), expected_output);
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
