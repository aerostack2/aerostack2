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
 * @file gates_detection.cpp
 *
 * GatesDetection class implementation
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "gate_detection/utils/arducam_interface.hpp"
#include <filesystem>

namespace gate_detection
{

ArducamInterface::ArducamInterface(
  as2::Node * node_ptr)
: node_ptr_(node_ptr), output_queue_(node_ptr, "arducam.output")
{
  camera_ =
    std::make_shared<as2::sensors::Camera>(node_ptr_, "", -1.0f);
  // We only use this to build TF, we don't publish nor update data

  if (!camera_) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "ArducamInterface: Could not create Camera object");
  }

  if (!setupCamera()) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "ArducamInterface: Could not setup the camera");
  }

  cameraInfoSetup();

  publish_images_ = getParameter<bool>(node_ptr_, "arducam.publish_images");
  publish_rectified_images_ =
    getParameter<bool>(node_ptr_, "arducam.publish_rectified_images");

  if (publish_rectified_images_) {
    // Publisher for rectified images
    rectified_image_pub_ =
      node_ptr_->create_publisher<sensor_msgs::msg::CompressedImage>(
      node_ptr_->generate_local_name(
        camera_name_ + "/rectified/image/compressed"), as2_names::topics::sensor_measurements::qos);
    // Publisher for rectified camera info
    rectified_camera_info_pub_ =
      node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(
      node_ptr_->generate_local_name(
        camera_name_ + "/rectified/camera_info"), as2_names::topics::sensor_measurements::qos);
  }

  int64_t miliseconds_from_framerate = static_cast<int64_t>(1000.0 / framerate_);

  static auto image_capture_timer_ = node_ptr_->create_timer(
    std::chrono::milliseconds(miliseconds_from_framerate),
    std::bind(&ArducamInterface::getFrame, this));

  // Callback group for the service
  image_capture_callback_group_ = node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create service for setting image save path
  set_image_path_srv_ = node_ptr_->create_service<as2_msgs::srv::SetImagePath>(
    node_ptr_->generate_local_name("set_image_save_path"),
    std::bind(
      &ArducamInterface::setImagePathCallback, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    image_capture_callback_group_);

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "Image save path service created: %s",
    node_ptr_->generate_local_name("set_image_save_path").c_str());

  // Create callback group for image saver timer (parallel execution)
  image_saver_callback_group_ = node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Start background image saving timer (runs at 100Hz to check queue frequently)
  image_saver_timer_ = node_ptr_->create_timer(
    std::chrono::milliseconds(10),  // 100Hz check rate
    std::bind(&ArducamInterface::imageSaverTimerCallback, this),
    image_saver_callback_group_);

  last_save_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_ptr_->get_logger(), "Image saver timer started");
}

ArducamInterface::~ArducamInterface()
{
  // Cancel the timer
  if (image_saver_timer_) {
    image_saver_timer_->cancel();
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Image saver timer stopped");
}

sensor_msgs::msg::CameraInfo ArducamInterface::getCameraInfoMessage() const
{
  return camera_info_;
}

bool ArducamInterface::setupCamera()
{
  std::string device_port;
  double framerate;
  int image_width;
  int image_height;
  int exposure_range = 0;
  double exposure_recompensation = 0.0;
  bool aelock = true;
  bool awblock = false;
  int gain = 0;
  int wbmode = 1;
  int aeantibanding = 1;
  float saturation = 1.0;

  enable_rectification_ =
    getParameter<bool>(node_ptr_, "enable_rectification");
  use_cuda_ = getParameter<bool>(node_ptr_, "arducam.use_cuda");

  camera_name_ = getParameter<std::string>(node_ptr_, "camera_name");
  device_port = getParameter<std::string>(node_ptr_, "arducam.device");
  framerate = getParameter<double>(node_ptr_, "arducam.framerate");
  image_height = getParameter<int>(node_ptr_, "image_height");
  image_width = getParameter<int>(node_ptr_, "image_width");
  aelock = getParameter<bool>(node_ptr_, "arducam.aelock");
  exposure_range = getParameter<int>(node_ptr_, "arducam.exposure_range");
  exposure_recompensation = getParameter<double>(node_ptr_, "arducam.exposure_recompensation");
  wbmode = getParameter<int>(node_ptr_, "arducam.wbmode");
  gain = getParameter<int>(node_ptr_, "arducam.gain");
  awblock = getParameter<bool>(node_ptr_, "arducam.awblock");
  aeantibanding = getParameter<int>(node_ptr_, "arducam.aeantibanding");
  saturation = getParameter<float>(node_ptr_, "arducam.saturation");

  framerate_ = getParameter<double>(node_ptr_, "arducam.framerate");

  RCLCPP_INFO(node_ptr_->get_logger(), "Video device: %s", device_port.c_str());

  RCLCPP_INFO(node_ptr_->get_logger(), "Using arducam");
  std::string image_width_str = std::to_string(image_width);
  std::string image_height_str = std::to_string(image_height);
  int framerate_int = static_cast<int>(std::round(framerate));
  std::string framerate_str = std::to_string(framerate_int);

  // auto device_full_name = "nvarguscamerasrc sensor-id=" + device_port +
  //   " ! video/x-raw(memory:NVMM), width=(int)" + image_width_str + ", height=(int)" +
  //   image_height_str + ",format=(string)NV12, framerate=(fraction)" + framerate_str +
  //   "/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw,format=(string)BGR ! appsink drop=1";  // NOLINT

  auto device_full_name = std::string("nvarguscamerasrc") + " sensor-id=" + device_port;
  if (exposure_recompensation != 0.0) {
    device_full_name += " exposurecompensation=" + std::to_string(exposure_recompensation);
  }
  if (exposure_range != 0) {
    device_full_name += " exposuretimerange=\"" + std::to_string(exposure_range) + " " +
      std::to_string(exposure_range) + "\"";
  }
  if (gain != 0) {
    device_full_name += " gainrange=\"" + std::to_string(gain) + " " + std::to_string(gain) +
      "\"";
  }
  // " ee-mode=0" +
  // " aeantibanding=1" +
  device_full_name += " aelock=" + std::string(aelock ? "1" : "0") + " awblock=" + std::string(
    awblock ? "1" : "0");
  if (wbmode != 1) {
    device_full_name += " wbmode=" + std::to_string(wbmode);
  }
  device_full_name += " aeantibanding=" + std::to_string(aeantibanding);
  if (saturation != 1.0) {
    device_full_name += " saturation=" + std::to_string(saturation);
  }
  // " saturation=1.75" +
  device_full_name = device_full_name + " ! video/x-raw(memory:NVMM)" +
    ", width=(int)" + image_width_str +
    ", height=(int)" + image_height_str +
    ", format=(string)NV12" +
    ", framerate=(fraction)" + framerate_str + "/1" +
    " ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw,format=(string)BGR ! appsink drop=1";     // NOLINT

  // auto device_full_name = std::string("nvarguscamerasrc") +
  //   " sensor-id=" + device_port +
  //   " exposurecompensation=1" +
  //   " exposuretimerange=\"16666666 16666666\"" +
  //   " wbmode=3" +
  //   " gain=\"8 8\"" +
  //   " awblock=1" +
  //   " ee-mode=0" +
  //   " aeantibanding=1" +
  //   " aelock=1"
  //   " saturation=1.75" +
  //   " ! video/x-raw(memory:NVMM)" +
  //   ", width=(int)" + image_width_str +
  //   ", height=(int)" + image_height_str +
  //   ", format=(string)NV12" +
  //   ", framerate=(fraction)" + framerate_str + "/1" +
  //   " ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw,format=(string)BGR ! appsink drop=1";  // NOLINT

  RCLCPP_INFO(node_ptr_->get_logger(), "Device full name: %s", device_full_name.c_str());
  cap_ = cv::VideoCapture(device_full_name, cv::CAP_GSTREAMER);
  // cap_.set(cv::CAP_PROP_EXPOSURE, 0);

  if (!cap_.isOpened()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Cannot open device");
    return false;
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Camera capture setup complete");

  return true;
}

void ArducamInterface::cameraInfoSetup()
{
  camera_info_.height = getParameter<int>(node_ptr_, "image_height");
  camera_info_.width = getParameter<int>(node_ptr_, "image_width");
  camera_info_.distortion_model = getParameter<std::string>(node_ptr_, "distortion_model");
  auto k = getParameter<std::vector<double>>(node_ptr_, "camera_matrix.data");
  convertVectorToArray(k, camera_info_.k);
  auto p = getParameter<std::vector<double>>(node_ptr_, "projection_matrix.data");
  convertVectorToArray(p, camera_info_.p);
  auto r = getParameter<std::vector<double>>(node_ptr_, "rectification_matrix.data");
  convertVectorToArray(r, camera_info_.r);
  camera_info_.d = getParameter<std::vector<double>>(
    node_ptr_,
    "distortion_coefficients.data");
  // Extract camera matrix K (3x3)
  const auto & K = camera_info_.k;
  if (K.size() == 9) {
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      K[0], K[1], K[2],
      K[3], K[4], K[5],
      K[6], K[7], K[8]);
  } else {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "GatesDetection: CameraInfo.K has size %zu (expected 9).", K.size());
  }

  // Extract distortion coefficients D
  const auto & D = camera_info_.d;
  if (!D.empty()) {
    dist_coeffs_ = cv::Mat::zeros(1, static_cast<int>(D.size()), CV_64F);
    for (int i = 0; i < static_cast<int>(D.size()); ++i) {
      dist_coeffs_.at<double>(0, i) = D[i];
    }
  } else {
    // No distortion provided -> assume zero distortion
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "GatesDetection: CameraInfo.D is empty. Assuming zero distortion.");
  }

  // Store distortion model
  distortion_model_ = camera_info_.distortion_model;

  // Check if we should enable rectification (only if there's actual distortion)
  significant_distorsion_ = !D.empty() && cv::norm(dist_coeffs_) > 1e-6;

  if (significant_distorsion_ && enable_rectification_) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "GatesDetection: Significant distorsion found. Distortion model: %s, D size: %zu",
      distortion_model_.c_str(), D.size());
  } else {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "GatesDetection: Image rectification disabled (no significant distortion)");
  }

  // Unsubscribe after receiving first message
  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "GatesDetection: Unsubscribing from camera info topic after first message.");

  if (enable_rectification_) {
    initRectificationMaps(
      cv::Size(camera_info_.width, camera_info_.height));
  }
}

void ArducamInterface::rectifyFrame(const cv::Mat & input_image, cv::Mat & output_image)
{
  // Convert ROS image message to OpenCV Mat
  try {
    // Apply image rectification if camera info is available and rectification is enabled
    if (enable_rectification_ && significant_distorsion_ && !camera_matrix_.empty() &&
      !dist_coeffs_.empty())
    {
      try {
        cv::cuda::GpuMat d_src, d_dst;
        d_src.upload(input_image);

        cv::cuda::remap(
          d_src, d_dst, d_map1_, d_map2_,
          cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        d_dst.download(output_image);
      } catch (const cv::Exception & e) {
        RCLCPP_ERROR(
          node_ptr_->get_logger(),
          "Failed to rectify image: %s. Using original image.", e.what());
        output_image = input_image;
      }
    } else {
      // No rectification needed or available
      output_image = input_image;
    }
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
}

bool ArducamInterface::stopCamera()
{
  return true;
}
bool ArducamInterface::isCameraRunning() const
{
  return true;
}

void ArducamInterface::initRectificationMaps(const cv::Size & size)
{
  if (camera_matrix_.empty() || dist_coeffs_.empty()) {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Cannot initialize rectification maps: missing camera matrix or distortion coefficients");
    rect_maps_initialized_ = false;
    return;
  }

  rectified_size_ = size;

  // Matriz de rectificación y nueva K (puedes ajustar si quieres cambiar FOV)
  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat P;

  if (distortion_model_ == "fisheye" || distortion_model_ == "equidistant") {
    // Modelo de ojo de pez - use estimateNewCameraMatrixForUndistortRectify with balance 0.0
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
      camera_matrix_, dist_coeffs_, rectified_size_, R, P, 0.0);

    cv::fisheye::initUndistortRectifyMap(
      camera_matrix_, dist_coeffs_,
      R, P,
      rectified_size_,
      CV_32FC1,   // map1 y map2 en float
      map1_, map2_);
    RCLCPP_INFO(node_ptr_->get_logger(), "Initialized fisheye rectification maps (CPU)");

    // Sustituit camera_matrix_ por P para usar la nueva cámara rectificada
    camera_matrix_ = P(cv::Rect(0, 0, 3, 3)).clone();
    dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
    camera_info_.k[0] = camera_matrix_.at<double>(0, 0);
    camera_info_.k[2] = camera_matrix_.at<double>(0, 2);
    camera_info_.k[4] = camera_matrix_.at<double>(1, 1);
    camera_info_.k[5] = camera_matrix_.at<double>(1, 2);
    camera_info_.p[0] = camera_matrix_.at<double>(0, 0);
    camera_info_.p[2] = camera_matrix_.at<double>(0, 2);
    camera_info_.p[5] = camera_matrix_.at<double>(1, 1);
    camera_info_.p[6] = camera_matrix_.at<double>(1, 2);
  } else if (distortion_model_ == "plumb_bob" ||
    distortion_model_ == "radtan" ||
    distortion_model_.empty())
  {
    // Modelo pinhole estándar
    cv::initUndistortRectifyMap(
      camera_matrix_, dist_coeffs_,
      R, P,
      rectified_size_,
      CV_32FC1,
      map1_, map2_);
    RCLCPP_INFO(node_ptr_->get_logger(), "Initialized plumb_bob rectification maps (CPU)");
  } else {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Unknown distortion model '%s'. Skipping rectification maps init.",
      distortion_model_.c_str());
    rect_maps_initialized_ = false;
    return;
  }
  d_map1_.upload(map1_);
  d_map2_.upload(map2_);
  rect_maps_initialized_ = true;
}

void ArducamInterface::getFrame()
{
  // Capture image in device with opencv2
  // time this function
  const auto framerate = std::chrono::high_resolution_clock::now();

  cv::Mat frame;
  if (!cap_.read(frame)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Cannot read image");
    return;
  }

  // Get timestamp for this frame
  auto timestamp = node_ptr_->now();

  // Queue image for saving if enabled (non-blocking)
  if (save_images_enabled_ && !image_save_path_.empty()) {
    std::string timestamp_str = std::to_string(timestamp.nanoseconds());
    std::string filename = image_save_path_ + "/" + timestamp_str + ".bmp";

    ImageToSave img_to_save;
    img_to_save.image = frame.clone();  // Clone the image for thread safety
    img_to_save.filename = filename;

    size_t queue_size;
    {
      std::lock_guard<std::mutex> lock(save_queue_mutex_);
      save_queue_.push(img_to_save);
      queue_size = save_queue_.size();
    }

    // Warn if queue is backing up
    if (queue_size > 10) {
      RCLCPP_WARN_THROTTLE(
        node_ptr_->get_logger(),
        *node_ptr_->get_clock(), 1000,
        "Image save queue backing up: %zu images pending", queue_size);
    }
  }

  if (publish_images_) {
    camera_->updateData(frame);
  }
  if (enable_rectification_) {
    rectifyFrame(frame, frame);
  }
  if (publish_rectified_images_) {
    auto rectified_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(),
      "bgr8",
      frame).toCompressedImageMsg();
    rectified_msg->header.stamp = node_ptr_->now();
    rectified_msg->header.frame_id = as2::tf::generateTfName(
      node_ptr_->get_namespace(), camera_name_ + '/' + "camera_link");
    rectified_image_pub_->publish(*rectified_msg);
    // Publish rectified camera info
    camera_info_.header = rectified_msg->header;
    rectified_camera_info_pub_->publish(camera_info_);
  }

  ArducamFrame arducam_frame;
  arducam_frame.image = frame;
  arducam_frame.header.stamp = node_ptr_->now();
  arducam_frame.header.frame_id = as2::tf::generateTfName(
    node_ptr_->get_namespace(), camera_name_ + '/' + "camera_link");

  const auto framerate_end = std::chrono::high_resolution_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    framerate_end - framerate).count();
  RCLCPP_DEBUG(
    node_ptr_->get_logger(),
    "ArducamInterface::getFrame() took %d fps", 1000.0 / (double)elapsed);
  output_queue_.push(arducam_frame);
}

void ArducamInterface::setImagePathCallback(
  const std::shared_ptr<as2_msgs::srv::SetImagePath::Request> request,
  std::shared_ptr<as2_msgs::srv::SetImagePath::Response> response)
{
  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "Received request to set image save path to: %s",
    request->path.c_str());
  if (request->path.empty()) {
    // Disable image saving
    save_images_enabled_ = false;
    image_save_path_.clear();
    response->success = true;
    RCLCPP_INFO(node_ptr_->get_logger(), "Image saving disabled");
  } else {
    // Check if directory exists, create if not
    std::filesystem::path save_dir(request->path);
    try {
      if (!std::filesystem::exists(save_dir)) {
        std::filesystem::create_directories(save_dir);
        RCLCPP_INFO(
          node_ptr_->get_logger(),
          "Created directory: %s", request->path.c_str());
      }

      // Validate that we can write to this directory
      if (!std::filesystem::is_directory(save_dir)) {
        response->success = false;
        RCLCPP_ERROR(
          node_ptr_->get_logger(),
          "Path exists but is not a directory: %s", request->path.c_str());
        return;
      }

      // Enable image saving
      image_save_path_ = request->path;
      save_images_enabled_ = true;
      response->success = true;
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Image saving enabled to: %s", request->path.c_str());
    } catch (const std::filesystem::filesystem_error & e) {
      response->success = false;
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Failed to create/access directory: %s", e.what());
    }
  }
}

void ArducamInterface::imageSaverTimerCallback()
{
  ImageToSave img_to_save;
  bool has_image = false;

  // Try to get an image from the queue
  {
    std::lock_guard<std::mutex> lock(save_queue_mutex_);
    if (!save_queue_.empty()) {
      img_to_save = save_queue_.front();
      save_queue_.pop();
      has_image = true;
    }
  }

  // Save the image (outside the lock to avoid blocking the queue)
  if (has_image && !img_to_save.image.empty()) {
    auto save_start = std::chrono::steady_clock::now();

    try {
      // Save as BMP (uncompressed) for maximum speed
      // No compression = fastest possible save, but larger files
      if (cv::imwrite(img_to_save.filename, img_to_save.image)) {
        auto save_end = std::chrono::steady_clock::now();
        auto save_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          save_end - save_start).count();

        // Calculate Hz (time since last save)
        auto time_since_last_save = std::chrono::duration_cast<std::chrono::milliseconds>(
          save_end - last_save_time_).count();
        double save_hz = time_since_last_save > 0 ? 1000.0 / time_since_last_save : 0.0;
        last_save_time_ = save_end;

        // Update running average of save time
        images_saved_count_++;
        avg_save_time_ms_ = avg_save_time_ms_ +
          (save_duration_ms - avg_save_time_ms_) / images_saved_count_;

        RCLCPP_DEBUG(
          node_ptr_->get_logger(),
          "Saved image: %s (took %.1f ms, %.1f Hz, avg: %.1f ms)",
          img_to_save.filename.c_str(),
          static_cast<double>(save_duration_ms),
          save_hz,
          avg_save_time_ms_);

        // Log performance summary periodically
        if (images_saved_count_ % 50 == 0) {
          size_t queue_size;
          {
            std::lock_guard<std::mutex> lock(save_queue_mutex_);
            queue_size = save_queue_.size();
          }
          RCLCPP_INFO(
            node_ptr_->get_logger(),
            "Image saving stats: %.1f Hz, avg save time: %.1f ms, queue: %zu",
            save_hz, avg_save_time_ms_, queue_size);
        }
      } else {
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Failed to save image: %s", img_to_save.filename.c_str());
      }
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "OpenCV exception while saving image: %s", e.what());
    }
  }
}

}  // namespace gate_detection
