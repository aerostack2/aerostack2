// Copyright 2024 Universidad Politécnica de Madrid
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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
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
* @file nominal_remap.cpp
*
* @brief Remap of camera frames from a calibrated camera to a nominal one
*/

#include "as2_usb_camera_interface/nominal_remap.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#if defined(AS2_USB_CAMERA_INTERFACE_NPP)
#include <cuda_runtime.h>
#include <npp.h>
#endif

namespace usb_camera_interface
{

namespace
{

/**
 * @brief Remap maps (float) from an ideal equidistant fisheye (nominal_matrix, no distortion: the
 * radius in normalized coordinates is the angle theta off the optical axis) to an equidistant
 * fisheye with distortion (theta_d = theta * (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 +
 * k4 theta^8)).
 *
 * Angles past the one where theta_d stops growing map outside the image: beyond it the polynomial
 * folds back, and would bring rays from behind the camera into the image.
 */
void equidistantToNominalMaps(
  const cv::Matx33d & camera_matrix, const std::vector<double> & distortion,
  const cv::Matx33d & nominal_matrix, const cv::Size & size, cv::Mat & map_x, cv::Mat & map_y)
{
  double k[4] = {0.0, 0.0, 0.0, 0.0};
  for (size_t i = 0; i < 4 && i < distortion.size(); ++i) {
    k[i] = distortion[i];
  }
  const auto theta_d = [&k](double theta) {
      const double t2 = theta * theta;
      return theta * (1.0 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))));
    };
  constexpr double step = 1e-4;
  double theta_max = CV_PI;
  for (double theta = 0.0; theta < CV_PI; theta += step) {
    if (theta_d(theta + step) <= theta_d(theta)) {
      theta_max = theta;
      break;
    }
  }

  const cv::Matx33d nominal_inverse = nominal_matrix.inv();
  map_x.create(size, CV_32FC1);
  map_y.create(size, CV_32FC1);
  map_x.setTo(cv::Scalar(-100.0f));
  map_y.setTo(cv::Scalar(-100.0f));
  for (int v = 0; v < size.height; ++v) {
    for (int u = 0; u < size.width; ++u) {
      const cv::Vec3d ray = nominal_inverse * cv::Vec3d(u, v, 1.0);
      const double theta = std::hypot(ray[0], ray[1]);
      if (theta > theta_max) {
        continue;
      }
      const double scale = theta > 1e-12 ? theta_d(theta) / theta : 1.0;
      const double x = ray[0] * scale;
      const double y = ray[1] * scale;
      map_x.at<float>(v, u) = static_cast<float>(
        camera_matrix(0, 0) * x + camera_matrix(0, 1) * y + camera_matrix(0, 2));
      map_y.at<float>(v, u) = static_cast<float>(camera_matrix(1, 1) * y + camera_matrix(1, 2));
    }
  }
}

}  // namespace

#if defined(AS2_USB_CAMERA_INTERFACE_NPP)

namespace
{

void check(cudaError_t error, const char * what)
{
  if (error != cudaSuccess) {
    throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(error));
  }
}

void check(NppStatus status, const char * what)
{
  if (status < 0) {  // positive values are warnings
    throw std::runtime_error(std::string(what) + ": NPP error " + std::to_string(status));
  }
}

}  // namespace

// GPU remap with NPP (CUDA toolkit), on 8-bit frames with 1 or 3 channels. Errors throw
// std::runtime_error.
struct NominalRemap::Gpu
{
  NppStreamContext context{};
  NppiSize size{};          // nominal image size
  Npp32f * map_x = nullptr;
  Npp32f * map_y = nullptr;
  int map_x_step = 0;
  int map_y_step = 0;
  // Frame buffers, (re)allocated for the frames' size and channels
  NppiSize frame_size{};
  int channels = 0;
  Npp8u * frame = nullptr;
  Npp8u * remapped = nullptr;
  int frame_step = 0;
  int remapped_step = 0;

  Gpu(const cv::Mat & host_map_x, const cv::Mat & host_map_y)
  {
    int devices = 0;
    if (cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0) {
      throw std::runtime_error("no CUDA device");
    }
    check(nppGetStreamContext(&context), "nppGetStreamContext");
    size = {host_map_x.cols, host_map_x.rows};
    map_x = nppiMalloc_32f_C1(size.width, size.height, &map_x_step);
    map_y = nppiMalloc_32f_C1(size.width, size.height, &map_y_step);
    if (map_x == nullptr || map_y == nullptr) {
      throw std::runtime_error("cannot allocate the maps on the GPU");
    }
    check(
      cudaMemcpy2D(
        map_x, map_x_step, host_map_x.data, host_map_x.step, size.width * sizeof(Npp32f),
        size.height, cudaMemcpyHostToDevice), "upload map_x");
    check(
      cudaMemcpy2D(
        map_y, map_y_step, host_map_y.data, host_map_y.step, size.width * sizeof(Npp32f),
        size.height, cudaMemcpyHostToDevice), "upload map_y");
  }

  ~Gpu()
  {
    for (void * buffer : {static_cast<void *>(map_x), static_cast<void *>(map_y),
        static_cast<void *>(frame), static_cast<void *>(remapped)})
    {
      if (buffer != nullptr) {
        nppiFree(buffer);
      }
    }
  }

  void allocateFrames(const cv::Mat & input)
  {
    if (input.cols == frame_size.width && input.rows == frame_size.height &&
      input.channels() == channels)
    {
      return;
    }
    nppiFree(frame);
    nppiFree(remapped);
    frame_size = {input.cols, input.rows};
    channels = input.channels();
    if (channels == 3) {
      frame = nppiMalloc_8u_C3(frame_size.width, frame_size.height, &frame_step);
      remapped = nppiMalloc_8u_C3(size.width, size.height, &remapped_step);
    } else {
      frame = nppiMalloc_8u_C1(frame_size.width, frame_size.height, &frame_step);
      remapped = nppiMalloc_8u_C1(size.width, size.height, &remapped_step);
    }
    if (frame == nullptr || remapped == nullptr) {
      throw std::runtime_error("cannot allocate the frames on the GPU");
    }
  }

  void remap(const cv::Mat & input, cv::Mat & output)
  {
    if (input.depth() != CV_8U || (input.channels() != 1 && input.channels() != 3)) {
      throw std::runtime_error("only 8-bit frames with 1 or 3 channels are remapped with NPP");
    }
    allocateFrames(input);
    const cudaStream_t stream = context.hStream;
    check(
      cudaMemcpy2DAsync(
        frame, frame_step, input.data, input.step, frame_size.width * channels,
        frame_size.height, cudaMemcpyHostToDevice, stream), "upload frame");
    // NPP leaves the pixels mapped outside the frame untouched: clear them to black
    check(
      cudaMemset2DAsync(remapped, remapped_step, 0, size.width * channels, size.height, stream),
      "clear output");
    const NppiRect frame_roi{0, 0, frame_size.width, frame_size.height};
    if (channels == 3) {
      check(
        nppiRemap_8u_C3R_Ctx(
          frame, frame_size, frame_step, frame_roi, map_x, map_x_step, map_y, map_y_step,
          remapped, remapped_step, size, NPPI_INTER_LINEAR, context), "nppiRemap_8u_C3R");
    } else {
      check(
        nppiRemap_8u_C1R_Ctx(
          frame, frame_size, frame_step, frame_roi, map_x, map_x_step, map_y, map_y_step,
          remapped, remapped_step, size, NPPI_INTER_LINEAR, context), "nppiRemap_8u_C1R");
    }
    output.create(size.height, size.width, input.type());
    check(
      cudaMemcpy2DAsync(
        output.data, output.step, remapped, remapped_step, size.width * channels, size.height,
        cudaMemcpyDeviceToHost, stream), "download remapped frame");
    check(cudaStreamSynchronize(stream), "synchronize");
  }
};

#else

// No GPU backend in this build: setup() never creates one
struct NominalRemap::Gpu
{
  void remap(const cv::Mat &, cv::Mat &) {}
};

#endif

NominalRemap::NominalRemap() = default;
NominalRemap::~NominalRemap() = default;

const char * NominalRemap::gpuBackend()
{
#if defined(AS2_USB_CAMERA_INTERFACE_NPP)
  return "NPP";
#else
  return "none";
#endif
}

bool NominalRemap::setup(
  const cv::Matx33d & camera_matrix, const std::vector<double> & distortion,
  const std::string & distortion_model, const cv::Matx33d & nominal_matrix,
  const cv::Size & size, bool use_gpu, std::string & message)
{
  message.clear();
  gpu_.reset();
  gpu_error_.clear();
  const bool fisheye = distortion_model == "equidistant" || distortion_model == "fisheye";
  if (!fisheye && distortion_model != "plumb_bob" && distortion_model != "rational_polynomial") {
    message = "Distortion model '" + distortion_model + "' is not supported for the nominal " +
      "camera (equidistant, fisheye, plumb_bob or rational_polynomial)";
    return false;
  }

  cv::Mat map_x, map_y;
  if (fisheye) {
    equidistantToNominalMaps(camera_matrix, distortion, nominal_matrix, size, map_x, map_y);
  } else {
    // Each nominal (pinhole) pixel's ray, distorted and projected by the calibrated camera
    cv::initUndistortRectifyMap(
      camera_matrix, distortion, cv::noArray(), nominal_matrix, size, CV_32FC1, map_x, map_y);
  }
  cv::convertMaps(map_x, map_y, map1_, map2_, CV_16SC2);

  if (!use_gpu) {
    return true;
  }
#if defined(AS2_USB_CAMERA_INTERFACE_NPP)
  try {
    gpu_ = std::make_unique<Gpu>(map_x, map_y);
    // Run the remap once now, so that a GPU that cannot run it falls back here and not mid-flight
    cv::Mat probe(16, 16, CV_8UC3, cv::Scalar::all(0)), probe_remapped;
    gpu_->remap(probe, probe_remapped);
  } catch (const std::exception & error) {
    gpu_.reset();
    message = std::string("remap_with_cuda: the GPU cannot run the remap with ") + gpuBackend() +
      " (" + error.what() + "), remapping on the CPU";
  }
#else
  message = "remap_with_cuda: built without NPP (CUDA toolkit), remapping on the CPU";
#endif
  return true;
}

void NominalRemap::apply(const cv::Mat & frame, cv::Mat & remapped)
{
  if (gpu_) {
    try {
      gpu_->remap(frame, remapped);
      return;
    } catch (const std::exception & error) {
      gpu_error_ = std::string("remap_with_cuda: the GPU remap with ") + gpuBackend() +
        " failed (" + error.what() + "), remapping on the CPU from now on";
      gpu_.reset();
    }
  }
  cv::remap(frame, remapped, map1_, map2_, cv::INTER_LINEAR);
}

}  // namespace usb_camera_interface
