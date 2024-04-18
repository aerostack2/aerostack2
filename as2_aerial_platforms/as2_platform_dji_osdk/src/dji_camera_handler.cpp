#include "dji_camera_handler.hpp"
#include <opencv2/highgui.hpp>

void camera_cb(CameraRGBImage pImg, void* userData) {
  std::cout << "camera_cb" << std::endl;
  auto img =
      cv::Mat(pImg.height, pImg.width, CV_8UC3, (void*)pImg.rawData.data());
  // create fullscreen window
  cv::imshow("camera", img);
  cv::waitKey(1);
};

void main_camera_cb(CameraRGBImage pImg, void* userData) {
  static bool first = true;
  // std::cout << "camera_cb" << std::endl;
  auto img =
      cv::Mat(pImg.height, pImg.width, CV_8UC3, (void*)pImg.rawData.data());
  // convert to RGB
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

  if (userData != nullptr) {
    as2::sensors::Camera* camera_ptr = (as2::sensors::Camera*)userData;
    camera_ptr->updateData(img);
  } else {
    // print original image size
    if (first) {
      std::cout << "original image size: " << img.size() << std::endl;
    }

    // create fullscreen window
    if (first) {
      cv::namedWindow("main_camera", cv::WINDOW_KEEPRATIO);
      // move to second screen
      cv::resizeWindow("main_camera", img.size().width, img.size().height);
      cv::moveWindow("main_camera", 1920, 0);
      first = false;
    }
    // cv::namedWindow("main_camera", cv::WINDOW_FULLSCREEN);
    cv::imshow("main_camera", img);
    cv::waitKey(1);
  }
};
