// Copyright 2026 Universidad Politécnica de Madrid
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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
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

/*!******************************************************************************
 *  \file       display_loader.hpp
 *  \brief      display loader implementation file.
 *  \authors    Asil Arnous
 ********************************************************************************/

#ifndef AS2_CAMERA_OVERLAY__DISPLAY_LOADER_HPP_
#define AS2_CAMERA_OVERLAY__DISPLAY_LOADER_HPP_
#include "headless_rviz.hpp"
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <string>
#include <vector>
namespace YAML {
class Node;
}
namespace as2_camera_overlay {
struct DisplayConfig {
  std::string class_id;
  std::string name;
};
class DisplayLoader {
public:
  explicit DisplayLoader(HeadlessDisplayContext *context,
                         rclcpp::Logger logger);
  ~DisplayLoader();
  bool loadDisplay(const DisplayConfig &cfg, const YAML::Node &properties);
  void updateAll(float wall_dt, float ros_dt);
  size_t size() const { return displays_.size(); }

private:
  static bool isExcluded(const std::string &class_id);
  static void applyProperties(rviz_common::properties::Property *prop,
                              const YAML::Node &yaml_map,
                              const rclcpp::Logger &logger);
  HeadlessDisplayContext *context_;
  rclcpp::Logger logger_;
  std::unique_ptr<pluginlib::ClassLoader<rviz_common::Display>> loader_;
  std::vector<std::shared_ptr<rviz_common::Display>> displays_;
};
} // namespace as2_camera_overlay
#endif
