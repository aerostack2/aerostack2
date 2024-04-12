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

/*!*******************************************************************************************
 *  \file       sensor_test.cpp
 *  \brief      Sensor class for AS2 header file
 *  \authors    Rafael Pérez Seguí
 *              Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 ********************************************************************************/

#include "as2_core/sensor.hpp"

#include "gtest/gtest.h"

namespace as2
{
namespace sensors
{

TEST(SensorTest, GenericSensor) {
  // Create an as2::Node
  as2::Node node("test_node");
  int pub_freq = 10;
  std::string topic_name = "my_topic_name";

  // Construct GenericSensor object
  EXPECT_NO_THROW(
    GenericSensor sensor = GenericSensor(topic_name, &node, pub_freq));
  pub_freq = -1;
  EXPECT_NO_THROW(
    GenericSensor sensor = GenericSensor(topic_name, &node, pub_freq));
  topic_name = as2_names::topics::sensor_measurements::gps;
  EXPECT_NO_THROW(
    GenericSensor sensor = GenericSensor(topic_name, &node, pub_freq));
  topic_name = "my_topic_name";
  EXPECT_NO_THROW(
    GenericSensor sensor = GenericSensor(topic_name, &node, pub_freq, false));

  // Destroy as2::Node
  node.~Node();
}

TEST(SensorTest, GroundTruth) {
  // Create an as2::Node
  as2::Node node("test_node");
  int pub_freq = 10;
  std::string topic_name_base = "my_base/";

  // Construct GroundTruth object
  EXPECT_NO_THROW(
    GroundTruth sensor_ground_truth = GroundTruth(
      &node, pub_freq, topic_name_base));

  GroundTruth sensor_ground_truth = GroundTruth(
    &node, pub_freq, topic_name_base);

  // Public methods
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::TwistStamped twist;
  EXPECT_NO_THROW(sensor_ground_truth.updateData(pose, twist));
  EXPECT_NO_THROW(sensor_ground_truth.updateData(pose));
  EXPECT_NO_THROW(sensor_ground_truth.updateData(twist));

  // Call GroundTruth destructor
  EXPECT_NO_THROW(sensor_ground_truth.~GroundTruth());

  // Destroy as2::Node
  node.~Node();
}

}  // namespace sensors
}  // namespace as2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
