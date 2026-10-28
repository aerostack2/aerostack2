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
* @file nominal_remap.hpp
*
* @brief Remap of camera frames from a calibrated camera to a nominal one
*/

#ifndef AS2_USB_CAMERA_INTERFACE__NOMINAL_REMAP_HPP_
#define AS2_USB_CAMERA_INTERFACE__NOMINAL_REMAP_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace usb_camera_interface
{

/**
 * @brief Remaps frames of a calibrated camera (matrix and distortion) to a nominal camera: the
 * nominal matrix and size with zero distortion, in the calibration's model. That is an ideal
 * equidistant fisheye for an equidistant (fisheye) calibration and an ideal pinhole for plumb_bob
 * or rational_polynomial.
 *
 * The remap runs on the CPU, or on the GPU with NPP (CUDA toolkit) when requested and the build
 * found NPP. The GPU state lives in the .cpp, so this class has the same layout whatever the
 * build.
 */
class NominalRemap
{
public:
  NominalRemap();
  ~NominalRemap();

  /**
   * @brief Build the remap maps.
   *
   * @param camera_matrix Calibrated camera matrix.
   * @param distortion Calibrated distortion coefficients.
   * @param distortion_model Calibrated distortion model.
   * @param nominal_matrix Nominal camera matrix.
   * @param size Nominal image size.
   * @param use_gpu Remap on the GPU. Falls back to the CPU when the build has no GPU backend or
   *                the GPU cannot run the remap, and says why in message.
   * @param message Output. Why there is no remap (on false), or why the GPU is not used.
   * @return false if the distortion model is not supported: no remap.
   */
  bool setup(
    const cv::Matx33d & camera_matrix, const std::vector<double> & distortion,
    const std::string & distortion_model, const cv::Matx33d & nominal_matrix,
    const cv::Size & size, bool use_gpu, std::string & message);

  /**
   * @brief Whether setup succeeded, so that frames are remapped.
   */
  bool enabled() const {return !map1_.empty();}

  /**
   * @brief Whether frames are remapped on the GPU.
   */
  bool onGpu() const {return gpu_ != nullptr;}

  /**
   * @brief The GPU backend of this build: "NPP" or "none".
   */
  static const char * gpuBackend();

  /**
   * @brief Remap a frame of the calibrated camera to the nominal camera. Only after a setup.
   *
   * If the GPU fails, this frame and the next ones are remapped on the CPU, onGpu() turns false
   * and gpuError() says why.
   *
   * @param frame Calibrated camera frame.
   * @param remapped Output. Nominal camera frame, black outside the calibrated camera's view.
   */
  void apply(const cv::Mat & frame, cv::Mat & remapped);

  /**
   * @brief Why the GPU stopped being used during apply(), empty if it did not.
   */
  const std::string & gpuError() const {return gpu_error_;}

private:
  struct Gpu;  // GPU (NPP) state, defined in nominal_remap.cpp

  // CPU maps (fixed point)
  cv::Mat map1_;
  cv::Mat map2_;
  std::unique_ptr<Gpu> gpu_;
  std::string gpu_error_;
};

}  // namespace usb_camera_interface

#endif  // AS2_USB_CAMERA_INTERFACE__NOMINAL_REMAP_HPP_
