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
* @file basic_server_gtest.cpp
*
* @brief Basic server test
*
* @author Miguel Fernández Cortizas
*         Pedro Arias Pérez
*         David Pérez Saura
*         Rafael Pérez Seguí
*/

#include <gtest/gtest.h>
#include <iostream>
#include "as2_behavior/behavior_server.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>
// #include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/action/takeoff.hpp"


class TakeoffServer : public as2_behavior::BehaviorServer<as2_msgs::action::Takeoff>
{
public:
  explicit TakeoffServer(const std::string & name)
  : as2_behavior::BehaviorServer<as2_msgs::action::Takeoff>(name)
  {
    std::cout << "TakeoffServer constructor" << std::endl;
  }
  int i = 0;
  bool on_activate(std::shared_ptr<const typename as2_msgs::action::Takeoff::Goal> goal) override
  {
    std::cout << "TakeoffServer activate" << std::endl;
    i = 0;
    return true;
  }
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::Takeoff::Goal> & goal,
    std::shared_ptr<as2_msgs::action::Takeoff::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::Takeoff::Result> & result_msg) override
  {
    feedback_msg->actual_takeoff_height = i++;
    if (i < 100) {
      return as2_behavior::ExecutionStatus::RUNNING;
    }

    result_msg->takeoff_success = true;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }
};

TEST(TakeoffServer, test_constructor) {
  EXPECT_NO_THROW(
    std::shared_ptr<TakeoffServer> node =
    std::make_shared<TakeoffServer>("TakeoffBehavior"));
}

TEST(TakeoffServer, test_activate) {
  std::shared_ptr<TakeoffServer> node =
    std::make_shared<TakeoffServer>("TakeoffBehavior");
  std::shared_ptr<as2_msgs::action::Takeoff::Goal> goal =
    std::make_shared<as2_msgs::action::Takeoff::Goal>();
  EXPECT_TRUE(node->on_activate(goal));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
