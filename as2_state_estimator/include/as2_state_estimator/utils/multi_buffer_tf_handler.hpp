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
* @file as2_state_estimator.hpp
*
* An utility for handling multiple tf buffers within the same tf_handler_
*
* @authors Miguel Fernández Cortizas
*
*
*/


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <as2_core/utils/tf_utils.hpp>


#ifndef AS2_STATE_ESTIMATOR__UTILS__AS2_STATE_ESTIMATOR_HPP_
#define AS2_STATE_ESTIMATOR__UTILS__AS2_STATE_ESTIMATOR_HPP_

namespace as2_state_estimator
{
/**
 * @brief A class that allows to handle multiple ways of interacting with the Tf buffer,
 * it has the purpose of select which transforms are wanted to be broadcasted to other
 * */
class ConfigurableTfHandler : public as2::tf::TfHandler
{
private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool local_buffer_ = false;
  as2::Node * node_ptr_;
  explicit MultiBufferTfHandler(
    as2::Node * node_ptr,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, bool local_buffer = false)
    : as2::tf::TfHandler(node_ptr), node_ptr_(node_ptr),
    tf_broadcaster_(tf_broadcaster)
  {}

  void useLocalBuffer(bool use_local_buffer)
  {
    local_buffer_ = use_local_buffer;
  }

  void publishTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    if (local_buffer_) {
      tf_buffer_->setTransform(transform, node_ptr_->get_name());
    } else {
      tf_broadcaster_->sendTransform(transform);
    }
  }
};
}  // namespace as2_state_estimator

#endif  // AS2_STATE_ESTIMATOR__AS2_STATE_ESTIMATOR_HPP_
