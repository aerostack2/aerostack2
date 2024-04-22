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

#include <std_msgs/msg/bool.hpp>
#include "gtest/gtest.h"

namespace as2
{
namespace sensors
{

TEST(SensorTest, TFStatic) {
  // Create an as2::Node
  auto node = std::make_shared<as2::Node>("test_tf_static_node");

  // Construct
  EXPECT_NO_THROW(TFStatic tf_static = TFStatic(node.get()));
  TFStatic tf_static = TFStatic(node.get());

  // Public methods
  std::string frame_id = "my_frame_id";
  std::string parent_frame_id = "my_parent_frame_id";

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = frame_id;
  transform.child_frame_id = parent_frame_id;
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  EXPECT_NO_THROW(tf_static.setStaticTransform(transform));
  EXPECT_NO_THROW(
    tf_static.setStaticTransform(
      frame_id, parent_frame_id, transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z, transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w));
  EXPECT_NO_THROW(
    tf_static.setStaticTransform(
      frame_id, parent_frame_id, transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z, transform.transform.rotation.x,
      transform.transform.rotation.y, transform.transform.rotation.z));
  EXPECT_NO_THROW(tf_static.getNode());


  // Spin node
  rclcpp::spin_some(node);
}

TEST(SensorTest, TFDynamic) {
  // Create an as2::Node
  auto node = std::make_shared<as2::Node>("test_tf_dynamic_node");

  // Construct
  EXPECT_NO_THROW(TFDynamic tf_dynamic = TFDynamic(node.get()));
  TFDynamic tf_dynamic = TFDynamic(node.get());

  // Public methods
  std::string frame_id = "my_frame_id";
  std::string parent_frame_id = "my_parent_frame_id";

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = frame_id;
  transform.child_frame_id = parent_frame_id;
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  EXPECT_NO_THROW(tf_dynamic.setDynamicTransform(transform));
  EXPECT_NO_THROW(
    tf_dynamic.setDynamicTransform(
      frame_id, parent_frame_id, transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z, transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w));
  EXPECT_NO_THROW(
    tf_dynamic.setDynamicTransform(
      frame_id, parent_frame_id, transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z, transform.transform.rotation.x,
      transform.transform.rotation.y, transform.transform.rotation.z));
  EXPECT_NO_THROW(
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster =
    tf_dynamic.getTransformBroadcaster());
  EXPECT_NO_THROW(tf_dynamic.getNode());

  // Spin node
  rclcpp::spin_some(node);
}

TEST(SensorTest, SensorData) {
  // Create an as2::Node
  auto node = std::make_shared<as2::Node>("test_generic_sensor_node");
  std::string topic_name = "my_topic_name";
  std::string sensor_name = "my_sensor_name";

  // Construct
  EXPECT_NO_THROW(
    SensorData sensor = SensorData<sensor_msgs::msg::Imu>("imu_topic", node.get()));
  EXPECT_NO_THROW(
    SensorData sensor =
    SensorData<sensor_msgs::msg::NavSatFix>("nav_sat_topic", node.get(), false));
  SensorData sensor = SensorData<std_msgs::msg::Bool>(topic_name, node.get());

  // Public methods
  topic_name = SensorData<std_msgs::msg::Bool>::processTopicName(sensor_name);
  std::string expected = as2_names::topics::sensor_measurements::base + sensor_name;
  EXPECT_EQ(topic_name, expected);
  topic_name = SensorData<std_msgs::msg::Bool>::processTopicName(sensor_name, false);
  EXPECT_EQ(topic_name, sensor_name);

  std_msgs::msg::Bool msg;
  EXPECT_NO_THROW(sensor.setData(msg));
  EXPECT_NO_THROW(sensor.publish());
  EXPECT_NO_THROW(sensor.updateAndPublish(msg));
  EXPECT_NO_THROW(std::string topic = sensor.getTopicName());
  EXPECT_NO_THROW(sensor.getData());

  // Spin node
  rclcpp::spin_some(node);
}

TEST(SensorTest, Sensor) {
  // Create an as2::Node
  auto node = std::make_shared<as2::Node>("test_sensor_node");
  std::string sensor_name = "my_sensor_name";
  float pub_freq = 10.0;
  double pub_freq_double = 10.0;
  int pub_freq_int = 10;
  bool add_sensor_measurements_base = true;

  // Construct
  EXPECT_NO_THROW(
    Sensor<sensor_msgs::msg::Imu> sensor =
    Sensor<sensor_msgs::msg::Imu>(sensor_name, node.get()));
  EXPECT_NO_THROW(
    Sensor<sensor_msgs::msg::Imu> sensor =
    Sensor<sensor_msgs::msg::Imu>(sensor_name, node.get(), pub_freq, add_sensor_measurements_base));
  EXPECT_NO_THROW(
    Sensor<sensor_msgs::msg::Imu> sensor =
    Sensor<sensor_msgs::msg::Imu>(
      sensor_name, node.get(), pub_freq_int,
      add_sensor_measurements_base));
  EXPECT_NO_THROW(
    Sensor<sensor_msgs::msg::Imu> sensor =
    Sensor<sensor_msgs::msg::Imu>(
      sensor_name, node.get(), pub_freq_double,
      add_sensor_measurements_base));
  Sensor<sensor_msgs::msg::Imu> sensor =
    Sensor<sensor_msgs::msg::Imu>(sensor_name, node.get());

  // Public methods
  sensor_msgs::msg::Imu imu_msg;
  EXPECT_NO_THROW(sensor.updateData(imu_msg));

  // Spin node
  rclcpp::spin_some(node);
}

TEST(SensorTest, Camera) {
  // Create an as2::Node
  auto node = std::make_shared<as2::Node>("test_camera_node");
  std::string sensor_name = "my_sensor_name";
  std::string info_name = "camera_info";
  std::string camera_link = "camera_link";

  float pub_freq = 10.0;
  bool add_sensor_measurements_base = true;

  // Construct
  EXPECT_NO_THROW(
    Camera sensor = Camera(sensor_name, node.get()));
  EXPECT_NO_THROW(
    Camera sensor =
    Camera(
      sensor_name, node.get(), pub_freq, add_sensor_measurements_base, info_name,
      camera_link));
  Camera sensor = Camera(sensor_name, node.get());

  // Public methods
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::CameraInfo camera_info;
  std::string encoding = "rgb8";
  EXPECT_NO_THROW(sensor.setParameters(camera_info, encoding));
  EXPECT_NO_THROW(
    sensor.setStaticTransform(
      "camera_link", "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
  EXPECT_NO_THROW(sensor.updateData(image));
  cv::Mat cv_image;
  EXPECT_NO_THROW(sensor.updateData(cv_image));

  // Spin node
  rclcpp::spin_some(node);
}

TEST(SensorTest, GroundTruth) {
  // Create an as2::Node
  auto node = std::make_shared<as2::Node>("test_ground_truth_node");
  float pub_freq = 10;
  std::string topic_name_base = "my_base/";

  // Construct
  EXPECT_NO_THROW(
    GroundTruth sensor_ground_truth = GroundTruth(node.get()));

  GroundTruth sensor_ground_truth = GroundTruth(
    node.get(), pub_freq, topic_name_base);

  // Public methods
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::TwistStamped twist;
  EXPECT_NO_THROW(sensor_ground_truth.updateData(pose, twist));
  EXPECT_NO_THROW(sensor_ground_truth.updateData(pose));
  EXPECT_NO_THROW(sensor_ground_truth.updateData(twist));

  // Spin node
  rclcpp::spin_some(node);
}

TEST(SensorTest, Gimbal) {
  // Create an as2::Node
  std::string gimbal_name = "my_gimbal";
  std::string gimbal_base_name = "my_gimbal_base";
  auto node = std::make_shared<as2::Node>("test_gimbal_node");
  float pub_freq = 10;
  bool add_sensor_measurements_base = true;

  // Construct
  EXPECT_NO_THROW(
    Gimbal sensor_gimbal = Gimbal(
      gimbal_name, gimbal_base_name, node.get()));
  EXPECT_NO_THROW(
    Gimbal sensor_gimbal = Gimbal(
      gimbal_name, gimbal_base_name, node.get(), pub_freq, add_sensor_measurements_base));

  Gimbal sensor_gimbal = Gimbal(
    gimbal_name, gimbal_base_name, node.get());

  // Public methods
  geometry_msgs::msg::Transform transform;
  std::string gimbal_parent_frame_id = "base_link";
  EXPECT_NO_THROW(sensor_gimbal.setGimbalBaseTransform(transform));
  EXPECT_NO_THROW(sensor_gimbal.setGimbalBaseTransform(transform, gimbal_parent_frame_id));
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_NO_THROW(sensor_gimbal.updateData(pose));
  EXPECT_NO_THROW(sensor_gimbal.getGimbalFrameId());
  EXPECT_NO_THROW(sensor_gimbal.getGimbalBaseFrameId());

  // Spin node
  rclcpp::spin_some(node);
}

}  // namespace sensors
}  // namespace as2

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
