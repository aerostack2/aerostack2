// Copyright 2025 Universidad Politécnica de Madrid
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

/*!*******************************************************************************************
 *  \file       as2_behaviors_object_perception.hpp
 *  \brief      Perception behavior manager header file.
 *  \authors    Alba López del Águila
 *  \copyright  Copyright (c) 2025 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef AS2_BEHAVIORS_OBJECT_PERCEPTION__AS2_BEHAVIORS_OBJECT_PERCEPTION_HPP_
#define AS2_BEHAVIORS_OBJECT_PERCEPTION__AS2_BEHAVIORS_OBJECT_PERCEPTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_core/names/topics.hpp"
#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/detect_objects.hpp"
#include "as2_behaviors_object_perception/detection_plugin_base.hpp"
#include "as2_behaviors_object_perception/common/img_preprocessing.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace as2_behaviors_object_perception
{

class PerceptionBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::DetectObjects>
{
public:
  explicit PerceptionBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PerceptionBehavior() {}

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image_msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg);

protected:
  bool on_activate(
    std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> goal) override;
  bool on_modify(
    std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> goal) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> & goal,
    std::shared_ptr<as2_msgs::action::DetectObjects::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::DetectObjects::Result> & result_msg) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  struct PipelineStage
  {
    std::string name;
    std::string plugin_name;
    std::string input_source;
    std::string input_stage;
    std::string input_topic;
    std::string output_topic;
    bool publish_output{false};
    std::shared_ptr<detection_plugin_base::DetectionBase> plugin;
    rclcpp::Publisher<as2_msgs::msg::ObjectPerceptionArray>::SharedPtr output_pub;
    rclcpp::Subscription<as2_msgs::msg::ObjectPerceptionArray>::SharedPtr input_sub;
    as2_msgs::msg::ObjectPerceptionArray external_input;
    bool has_external_input{false};
  };

  void loadPipeline();
  PipelineStage loadStage(const std::string & stage_name);
  void loadSinglePluginPipeline();
  PipelineStage * findStage(const std::string & stage_name);
  void publishStageOutput(const PipelineStage & stage);
  void external_input_callback(
    const std::string & stage_name,
    const as2_msgs::msg::ObjectPerceptionArray::SharedPtr msg);

  pluginlib::ClassLoader<detection_plugin_base::DetectionBase> detection_loader_;
  std::vector<PipelineStage> pipeline_stages_;

  // Centralized image preprocessing: decompression + optional rectification
  as2_behaviors_object_perception::ImagePreprocessor preprocessor_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  std::string camera_image_topic_;
  std::string camera_info_topic_;

  bool persistent_;
  std::string plugin_name_;
  as2_msgs::msg::ObjectPerceptionArray latest_pipeline_output_;
};

}  // namespace as2_behaviors_object_perception
#endif  // AS2_BEHAVIORS_OBJECT_PERCEPTION__AS2_BEHAVIORS_OBJECT_PERCEPTION_HPP_
