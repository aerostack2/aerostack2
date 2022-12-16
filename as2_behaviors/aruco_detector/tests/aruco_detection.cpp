#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "as2_aruco_detector.hpp"
#include "gtest/gtest.h"

TEST(ArucoDetector, Aruco_detection) {
  ArucoDetector aruco_detector;
  sensor_msgs::msg::Image::SharedPtr image;
  std::string image_path = "aruco_img.png";

  // load img in image_msg
  cv::Mat img = cv::imread(image_path);
  image       = std::make_shared<sensor_msgs::msg::Image>();
  cv_bridge::CvImage cv_img;
  cv_img.image    = img;
  cv_img.encoding = sensor_msgs::image_encodings::BGR8;
  cv_img.toImageMsg(*image);

  aruco_detector.imageCallback(image);
  // get the pose of the aruco marker
  // auto aruco_pose = aruco_detector.getAruco();
  // EXPECT_EQ(pose.id, 1);
  // EXPECT_EQ(pose.pose.position.x, 0.0);
  // EXPECT_EQ(pose.pose.position.y, 0.0);
  // EXPECT_EQ(pose.pose.position.z, 0.0);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
