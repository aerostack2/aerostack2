// Copyright 2023 Universidad Politécnica de Madrid
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
 * @file px4_odom_test.cpp
 *
 * PX4 odometry test.
 *
 * @author Miguel Fernández Cortizas
 *         Rafael Pérez Seguí
 *         Pedro Arias Pérez
 *         Javier Melero Deza
 */

#include <iomanip>
#include <iostream>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

void convert(tf2::Quaternion q_ned)
{
  tf2::Matrix3x3 ned2enu(0, 1, 0, 1, 0, 0, 0, 0, -1);
  tf2::Quaternion q_ned2enu;
  ned2enu.getRotation(q_ned2enu);

  tf2::Matrix3x3 frd2flu(1, 0, 0, 0, -1, 0, 0, 0, -1);
  tf2::Quaternion q_frd2flu;
  frd2flu.getRotation(q_frd2flu);

  tf2::Quaternion q_enu = q_ned2enu * q_ned * q_frd2flu;

  double roll, pitch, yaw;
  tf2::Matrix3x3(q_enu).getRPY(roll, pitch, yaw);

  std::cout << std::setprecision(2) << std::fixed;
  std::cout << "roll: " << roll * (180 / 3.1416) << ", pitch: " << pitch * (180 / 3.1416)
            << ", yaw: " << yaw * (180 / 3.1416) << std::endl;
  return;
}

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    px4_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/vehicle_odometry/out", rclcpp::SensorDataQoS(),
      std::bind(&MinimalSubscriber::px4odometryCallback, this, std::placeholders::_1));
  }

private:
  void px4odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    tf2::Quaternion q_msg(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
    convert(q_msg);
    return;
  }
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odometry_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
