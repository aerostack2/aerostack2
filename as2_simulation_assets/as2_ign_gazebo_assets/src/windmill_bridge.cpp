/*!*******************************************************************************************
 *  \file       ground_truth_bridge.cpp
 *  \brief      Ignition bridge ground truth implementation file.
 *  \authors    Javier Melero Deza
 *              Pedro Arias Pérez
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include <math.h>

#include <as2_core/names/topics.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ignition/transport/AdvertiseOptions.hh>
#include <ros_gz_bridge/convert.hpp>

class WindmillBridge: public rclcpp::Node {
public:
    WindmillBridge(): Node("windmill_bridge") {
        this->declare_parameter<std::string>("name_space");
        this->get_parameter("name_space", model_name_);
        this->declare_parameter<std::string>("world_name");
        this->get_parameter("world_name", world_name_);

        wm_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "motor/cmd_vel/angular_vel", as2_names::topics::actuator_command::qos,
            std::bind(&WindmillBridge::cmdCallback, this, std::placeholders::_1));

        // Initialize the ignition node
        ign_node_ptr_                  = std::make_shared<ignition::transport::Node>();
        std::string joint_force_topic = "/model/"+ model_name_ +"/joint/motor_link_joint/cmd_force";
        std::string joint_topic = "/world" + world_name_ + "/model/" + model_name_ + "/joint_state";

        ign_node_ptr_->Subscribe(joint_topic, this->ignitionJointStateCallback);

        ign_node_ptr_->Advertise<ignition::msgs::Double>(joint_force_topic);   
    }
private:
    std::shared_ptr<ignition::transport::Node> ign_node_ptr_;
    std::string model_name_;
    std::string world_name_;
    float joint_vel;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wm_sub_;

private:
    static void ignitionJointStateCallback(const ignition::msgs::Model &joint_state, const ignition::transport::MessageInfo &msg_info){

    }

    void cmdCallback(const std_msgs::msg::Float32::SharedPtr cmd_vel){

    }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WindmillBridge>());
  rclcpp::shutdown();
  return 0;
}