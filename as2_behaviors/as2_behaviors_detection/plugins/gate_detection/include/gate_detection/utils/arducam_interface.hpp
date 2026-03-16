// Copyright 2025 UNIVERSIDAD POLITÉCNICA DE MADRID
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
//    * Neither the name of the UNIVERSIDAD POLITÉCNICA DE MADRID nor the names of its
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
 * @file arducam_interface.hpp
 *
 * Te leo la arducam papito
 *
 * @author Javier Melero Deza
 */


#ifndef GATE_DETECTION__ARDUCAM_INTERFACE_HPP_
#define GATE_DETECTION__ARDUCAM_INTERFACE_HPP_

#include <string>
#include <queue>
#include <memory>
#include <vector>
#include <chrono>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudawarping.hpp>   // for cv::cuda::remap
#include <opencv2/cudaarithm.hpp>

#include <as2_core/node.hpp>
#include "as2_core/sensor.hpp"
#include "gate_detection/utils/common.hpp"
#include "as2_msgs/srv/set_image_path.hpp"


namespace gate_detection
{

struct ArducamFrame
{
  std_msgs::msg::Header header;
  cv::Mat image;
};

struct ImageToSave
{
  cv::Mat image;
  std::string filename;
};

class ArducamInterface
{
public:
  explicit ArducamInterface(as2::Node * node_ptr);

  virtual ~ArducamInterface();

  bool setupCamera();
  void cameraInfoSetup();
  sensor_msgs::msg::CameraInfo getCameraInfoMessage() const;
  bool stopCamera();
  bool isCameraRunning() const;
  void initRectificationMaps(const cv::Size & image_size);
  void getFrame();
  void rectifyFrame(const cv::Mat & input_image, cv::Mat & output_image);
  void setImagePathCallback(
    const std::shared_ptr<as2_msgs::srv::SetImagePath::Request> request,
    std::shared_ptr<as2_msgs::srv::SetImagePath::Response> response);
  void imageSaverTimerCallback();

  template<std::size_t N>
  bool convertVectorToArray(
    const std::vector<double> & vec, std::array<double, N> & array)
  {
    // Check if the vector has the same size as the array
    if (vec.size() != array.size()) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "The vector size is different from the array size. Vector size: %ld, Array size: %ld",
        vec.size(), array.size());
      return false;
    }

    // Copy the vector to the array
    std::memcpy(array.data(), vec.data(), array.size() * sizeof(double));
    return true;
  }

  MutexQueue<ArducamFrame> & getOutputQueue()
  {
    return output_queue_;
  }

private:
  double framerate_ = 30.0;
  bool publish_images_ = false;
  bool publish_rectified_images_ = false;
  std::string camera_name_;

  as2::Node * node_ptr_;
  MutexQueue<ArducamFrame> output_queue_;
  std::shared_ptr<as2::sensors::Camera> camera_;
  cv::VideoCapture cap_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rectified_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rectified_camera_info_pub_;

  rclcpp::Service<as2_msgs::srv::SetImagePath>::SharedPtr set_image_path_srv_;
  rclcpp::CallbackGroup::SharedPtr image_capture_callback_group_;

  sensor_msgs::msg::CameraInfo camera_info_;
  // Camera calibration for rectification
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::string distortion_model_;

  bool use_cuda_ = false;
  bool enable_rectification_ = false;
  bool significant_distorsion_ = false;

  // Unique initialization of rectification maps
  cv::Mat map1_, map2_;
  cv::cuda::GpuMat d_map1_, d_map2_;
  cv::Size rectified_size_;
  bool rect_maps_initialized_{false};

  // Image saving variables
  std::string image_save_path_;
  std::atomic<bool> save_images_enabled_{false};

  // Background image saving with ROS2 timer
  std::queue<ImageToSave> save_queue_;
  std::mutex save_queue_mutex_;
  rclcpp::TimerBase::SharedPtr image_saver_timer_;
  rclcpp::CallbackGroup::SharedPtr image_saver_callback_group_;

  // Performance metrics
  std::chrono::steady_clock::time_point last_save_time_;
  int images_saved_count_{0};
  double avg_save_time_ms_{0.0};
};
}  // namespace gate_detection

#endif  // GATE_DETECTION__ARDUCAM_INTERFACE_HPP_
