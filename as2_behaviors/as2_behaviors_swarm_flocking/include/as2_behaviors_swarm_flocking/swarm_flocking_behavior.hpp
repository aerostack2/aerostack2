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

/*!******************************************************************************
 *  \file       swarm_flocking_behavior.hpp
 *  \brief      Aerostack2 swarm_behavior header file.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************/

 #ifndef AS2_BEHAVIORS_SWARM_FLOCKING__SWARM_FLOCKING_BEHAVIOR_HPP_
 #define AS2_BEHAVIORS_SWARM_FLOCKING__SWARM_FLOCKING_BEHAVIOR_HPP_

 #include <memory>
 #include <string>
 #include <vector>
 #include <functional>
 #include <chrono>
 #include <algorithm>
 #include <unordered_map>
 #include <drone_swarm.hpp>


 #include <rclcpp/rclcpp.hpp>
 #include <rclcpp_action/rclcpp_action.hpp>
 #include "as2_behavior/behavior_server.hpp"
 #include "geometry_msgs/msg/pose_stamped.hpp"
 #include "as2_msgs/action/swarm_flocking.hpp"
 #include "as2_msgs/srv/modify_swarm.hpp"
 #include "as2_msgs/msg/pose_with_id_array.hpp"
 #include "as2_core/names/actions.hpp"
 #include "as2_core/utils/frame_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class SwarmFlockingBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::SwarmFlocking>
{
public:
  SwarmFlockingBehavior();
  ~SwarmFlockingBehavior() {}

  std::vector<std::shared_ptr<rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowReference>>>
  goal_future_handles_;

private:
  as2_msgs::action::SwarmFlocking::Goal goal_;
  as2_msgs::action::SwarmFlocking::Result result_;
  as2_msgs::action::SwarmFlocking::Feedback feedback_;
  std::shared_ptr<rclcpp::Service<as2_msgs::action::SwarmFlocking::Impl::SendGoalService>>
  modify_srv_ = nullptr;

  rclcpp::CallbackGroup::SharedPtr cbk_group_;
  std::unordered_map<std::string, std::shared_ptr<DroneSwarm>> drones_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_swarm_broadcaster_;
  std::shared_ptr<geometry_msgs::msg::TransformStamped> transform_;
  std::string swarm_base_link_frame_id_;

  // Suscriber to the dynamic swarm formation
  rclcpp::Subscription<as2_msgs::msg::PoseWithIDArray>::SharedPtr dynamic_swarm_formation_;

  // Tf to pause the behavior
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
  bool on_activate(
    std::shared_ptr<const as2_msgs::action::SwarmFlocking::Goal> goal) override;
  bool on_modify(std::shared_ptr<const as2_msgs::action::SwarmFlocking::Goal> goal) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::SwarmFlocking::Goal> & goal,
    std::shared_ptr<as2_msgs::action::SwarmFlocking::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::SwarmFlocking::Result> & result_msg) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  /**
 * @brief Set up the virtual centroid of the swarm with offset in the desired frame
 * @param virtual_centroid The virtual centroid of the swarm in the desired frame
 * @return bool Return true if the frame is not empty
 */
  bool setUpVirtualCentroid(const geometry_msgs::msg::PoseStamped & virtual_centroid);

  /**
* @brief Set the drones reference in the swarm
* @param centroid The centroid of the swarm
* @param drones_namespace The namespaced of the drones
* @param formation The position of the drones in the swarm
* @return bool Always true at the end of the function
*/
  bool setUpDronesFormation(
    geometry_msgs::msg::PoseStamped centroid,
    std::vector<std::string> drones_namespace, std::vector<as2_msgs::msg::PoseWithID> formation);

  /**
   * @brief Active the followReference of the drones and check if the drones are ready to
   * execute the action
   * @return bool Return true if the drones are ready to execute
   */
  bool initDroneReferences();


  /**
   * @brief Check the followReference status of the drones
   * @param goal_future_handles The goal future handles of the drones
   * @return as2_behavior::ExecutionStatus Return the status of the monitoring
   */
  as2_behavior::ExecutionStatus monitoring(
    const std::vector<std::shared_ptr<
      rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowReference>>>
    goal_future_handles);

  /**
   * @brief Callback to update the refrences of the drones inside the swarm
   * @param new_formation The new formation of the swarm
   */
  void dynamicSwarmFormationCallback(as2_msgs::msg::PoseWithIDArray new_formation);

  /**
   * @brief Service to modify the virtual_centroid, add or detach a new drone reference
   * within the swarm
   * @param request The request of the service
   * @param response The response of the service
   */
  void modifySwarmSrv(
    const std::shared_ptr<as2_msgs::action::SwarmFlocking::Impl::SendGoalService::Request>
    request,
    const std::shared_ptr<as2_msgs::action::SwarmFlocking::Impl::SendGoalService::Response>
    response);
};

#endif  // AS2_BEHAVIORS_SWARM_FLOCKING__SWARM_FLOCKING_BEHAVIOR_HPP_
