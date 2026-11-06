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
* @file raw_odometry.cpp
*
* Deprecated alias of the external_localization plugin, kept so existing raw_odometry configs
* keep working. It reads the raw_odometry parameter namespace and its odom_sub_topic key.
*/

#include <pluginlib/class_list_macros.hpp>
#include "external_localization/external_localization.hpp"

namespace raw_odometry
{

class Plugin : public external_localization::Plugin
{
public:
  Plugin()
  {
    prefix_ = "raw_odometry";
    odometry_topic_key_ = "odom_sub_topic";
  }

  void onSetup() override
  {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "raw_odometry is deprecated and will be removed in a future release. Rename the plugin "
      "to external_localization and move odom_sub_topic to odometry.topic");
    external_localization::Plugin::onSetup();
  }
};

}  // namespace raw_odometry

PLUGINLIB_EXPORT_CLASS(raw_odometry::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)
