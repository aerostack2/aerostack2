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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//      of its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
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
 * @file simple_ekf_utils_gtest.cpp
 *
 * Unit tests for the inline utility functions in simple_ekf_utils.hpp.
 * Tests: frame transforms, covariance config, measurement conversion,
 * angle unwrapping, twist computation.
 *
 * rclcpp::init/shutdown are called in main() because tf2_geometry_msgs
 * references the rclcpp time system.
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "simple_ekf/simple_ekf_utils.hpp"
#include "ekf/ekf_datatype.hpp"

using simple_ekf::PoseTopicConfig;
using simple_ekf::StateTransforms;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static geometry_msgs::msg::PoseWithCovarianceStamped makePoseMsg(
  const std::string & frame_id,
  double x, double y, double z,
  double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = frame_id;
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = z;
  msg.pose.pose.orientation.x = qx;
  msg.pose.pose.orientation.y = qy;
  msg.pose.pose.orientation.z = qz;
  msg.pose.pose.orientation.w = qw;
  return msg;
}

static PoseTopicConfig makeFixedConfig(
  double px, double py, double pz,
  double rx, double ry, double rz)
{
  PoseTopicConfig c;
  c.use_message_covariance = false;
  c.position_values = {px, py, pz};
  c.orientation_values = {rx, ry, rz};
  return c;
}

static PoseTopicConfig makeMultConfig(
  double mx, double my, double mz,
  double rx, double ry, double rz)
{
  PoseTopicConfig c;
  c.use_message_covariance = true;
  c.position_values = {mx, my, mz};
  c.orientation_values = {rx, ry, rz};
  return c;
}

// Fill a 36-element covariance array with diagonal values
static std::array<double, 36> makeDiagCov36(
  double px, double py, double pz,
  double rx, double ry, double rz)
{
  std::array<double, 36> cov = {};
  cov[0]  = px;  // x variance
  cov[7]  = py;  // y variance
  cov[14] = pz;  // z variance
  cov[21] = rx;  // roll variance
  cov[28] = ry;  // pitch variance
  cov[35] = rz;  // yaw variance
  return cov;
}

// Build a tf2::Transform from translation only
static tf2::Transform makeTranslation(double x, double y, double z)
{
  tf2::Transform t;
  t.setIdentity();
  t.setOrigin(tf2::Vector3(x, y, z));
  return t;
}

// Build a tf2::Transform from yaw rotation only (around Z)
static tf2::Transform makeYawRotation(double yaw)
{
  tf2::Transform t;
  t.setIdentity();
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  t.setRotation(q);
  return t;
}

// ---------------------------------------------------------------------------
// Transform round-trip tests
// ---------------------------------------------------------------------------

// Converts an Eigen 4x4 transform to a tf2::Transform and back; the result
// should match the original matrix (round-trip is lossless).
TEST(UtilsTransformTest, EigenToTf2RoundTrip)
{
  // Build a 4×4 with translation (1,2,3) and 90° yaw
  double yaw = M_PI / 2.0;
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m(0, 3) = 1.0;
  m(1, 3) = 2.0;
  m(2, 3) = 3.0;
  m(0, 0) = std::cos(yaw);  m(0, 1) = -std::sin(yaw);
  m(1, 0) = std::sin(yaw);  m(1, 1) =  std::cos(yaw);

  tf2::Transform tf = simple_ekf::eigenMatrix4dToTf2Transform(m);
  Eigen::Matrix4d result = simple_ekf::tf2TransformToEigenMatrix4d(tf);

  EXPECT_TRUE(result.isApprox(m, 1e-10));
}

// Converts a tf2::Transform to an Eigen 4x4 matrix and back; origin and
// rotation (compared via quaternion dot product) should be preserved.
TEST(UtilsTransformTest, Tf2ToEigenRoundTrip)
{
  tf2::Transform original;
  original.setOrigin(tf2::Vector3(4.0, -1.0, 2.5));
  tf2::Quaternion q;
  q.setRPY(0.1, 0.2, 0.3);
  original.setRotation(q);

  Eigen::Matrix4d m = simple_ekf::tf2TransformToEigenMatrix4d(original);
  tf2::Transform result = simple_ekf::eigenMatrix4dToTf2Transform(m);

  EXPECT_NEAR(result.getOrigin().x(), original.getOrigin().x(), 1e-10);
  EXPECT_NEAR(result.getOrigin().y(), original.getOrigin().y(), 1e-10);
  EXPECT_NEAR(result.getOrigin().z(), original.getOrigin().z(), 1e-10);
  // Compare quaternion dot product (handles sign ambiguity)
  double dot = std::abs(
    result.getRotation().dot(original.getRotation()));
  EXPECT_NEAR(dot, 1.0, 1e-10);
}

// Converting an identity Eigen matrix yields a tf2::Transform with zero
// translation and a unit quaternion.
TEST(UtilsTransformTest, EigenToTf2_Identity)
{
  tf2::Transform t = simple_ekf::eigenMatrix4dToTf2Transform(Eigen::Matrix4d::Identity());
  EXPECT_NEAR(t.getOrigin().x(), 0.0, 1e-12);
  EXPECT_NEAR(t.getOrigin().y(), 0.0, 1e-12);
  EXPECT_NEAR(t.getOrigin().z(), 0.0, 1e-12);
  EXPECT_NEAR(t.getRotation().w(), 1.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Covariance config tests
// ---------------------------------------------------------------------------

// With use_message_covariance = false, the configured position/orientation
// variances land on the diagonal (indices 0, 7, 14, 21, 28, 35) and every
// other element stays zero.
TEST(UtilsCovarianceTest, GenerateCovariance_FixedMode)
{
  auto config = makeFixedConfig(0.01, 0.02, 0.03, 0.001, 0.002, 0.003);
  auto cov = simple_ekf::generateCovarianceFromConfig(config);

  EXPECT_DOUBLE_EQ(cov[0],  0.01);
  EXPECT_DOUBLE_EQ(cov[7],  0.02);
  EXPECT_DOUBLE_EQ(cov[14], 0.03);
  EXPECT_DOUBLE_EQ(cov[21], 0.001);
  EXPECT_DOUBLE_EQ(cov[28], 0.002);
  EXPECT_DOUBLE_EQ(cov[35], 0.003);

  // All off-diagonal elements are zero
  for (int i = 0; i < 36; ++i) {
    if (i != 0 && i != 7 && i != 14 && i != 21 && i != 28 && i != 35) {
      EXPECT_DOUBLE_EQ(cov[i], 0.0) << "cov[" << i << "] should be 0";
    }
  }
}

// With use_message_covariance = true, generateCovarianceFromConfig is just
// a placeholder and returns an all-zero covariance.
TEST(UtilsCovarianceTest, GenerateCovariance_MultiplierMode_ReturnsZero)
{
  auto config = makeMultConfig(2.0, 2.0, 2.0, 1.0, 1.0, 1.0);
  auto cov = simple_ekf::generateCovarianceFromConfig(config);

  for (int i = 0; i < 36; ++i) {
    EXPECT_DOUBLE_EQ(cov[i], 0.0) << "cov[" << i << "] should be 0 in multiplier mode";
  }
}

// With use_message_covariance = false, getCovarianceWithConfig ignores the
// input covariance entirely and returns the fixed config values.
TEST(UtilsCovarianceTest, GetCovarianceWithConfig_FixedMode_IgnoresInput)
{
  std::array<double, 36> input;
  input.fill(99.0);  // should be completely ignored

  auto config = makeFixedConfig(0.01, 0.02, 0.03, 0.001, 0.002, 0.003);
  auto cov = simple_ekf::getCovarianceWithConfig(input, config);

  EXPECT_DOUBLE_EQ(cov[0],  0.01);
  EXPECT_DOUBLE_EQ(cov[7],  0.02);
  EXPECT_DOUBLE_EQ(cov[14], 0.03);
}

// With use_message_covariance = true, each diagonal element of the output
// equals the input covariance multiplied by the configured factor.
TEST(UtilsCovarianceTest, GetCovarianceWithConfig_MultiplierMode)
{
  auto input = makeDiagCov36(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
  auto config = makeMultConfig(2.0, 3.0, 4.0, 5.0, 6.0, 7.0);
  auto cov = simple_ekf::getCovarianceWithConfig(input, config);

  EXPECT_NEAR(cov[0],  0.1 * 2.0, 1e-12);
  EXPECT_NEAR(cov[7],  0.2 * 3.0, 1e-12);
  EXPECT_NEAR(cov[14], 0.3 * 4.0, 1e-12);
  EXPECT_NEAR(cov[21], 0.4 * 5.0, 1e-12);
  EXPECT_NEAR(cov[28], 0.5 * 6.0, 1e-12);
  EXPECT_NEAR(cov[35], 0.6 * 7.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Pose → EKF measurement conversion
// ---------------------------------------------------------------------------

// Position is copied through unchanged, and an identity orientation
// yields roll/pitch/yaw ~ 0.
TEST(UtilsMeasurementTest, PoseToEkfMeasurement_PositionExtraction)
{
  auto msg = makePoseMsg("map", 1.5, 2.5, 3.5);
  auto meas = simple_ekf::poseWithCovarianceToRawEkfMeasurement(msg);

  EXPECT_NEAR(meas.data[ekf::PoseMeasurement::X], 1.5, 1e-12);
  EXPECT_NEAR(meas.data[ekf::PoseMeasurement::Y], 2.5, 1e-12);
  EXPECT_NEAR(meas.data[ekf::PoseMeasurement::Z], 3.5, 1e-12);
  EXPECT_NEAR(meas.data[ekf::PoseMeasurement::ROLL],  0.0, 1e-9);
  EXPECT_NEAR(meas.data[ekf::PoseMeasurement::PITCH], 0.0, 1e-9);
  EXPECT_NEAR(meas.data[ekf::PoseMeasurement::YAW],   0.0, 1e-9);
}

// A quaternion encoding a 90° yaw is converted back to yaw = π/2 via
// tf2::Matrix3x3::getRPY.
TEST(UtilsMeasurementTest, PoseToEkfMeasurement_QuatToYaw)
{
  // Encode yaw = π/2 as quaternion
  double yaw = M_PI / 2.0;
  double qz = std::sin(yaw / 2.0);
  double qw = std::cos(yaw / 2.0);
  auto msg = makePoseMsg("map", 0.0, 0.0, 0.0, 0.0, 0.0, qz, qw);

  auto meas = simple_ekf::poseWithCovarianceToRawEkfMeasurement(msg);

  EXPECT_NEAR(meas.data[ekf::PoseMeasurement::YAW], M_PI / 2.0, 1e-9);
}

// The diagonal entries of a PoseWithCovariance are extracted into the
// matching X/Y/Z/ROLL/PITCH/YAW fields of ekf::PoseMeasurementCovariance.
TEST(UtilsMeasurementTest, PoseToEkfMeasurementCov_DiagonalExtraction)
{
  geometry_msgs::msg::PoseWithCovariance pose_cov;
  pose_cov.covariance[0]  = 0.01;
  pose_cov.covariance[7]  = 0.02;
  pose_cov.covariance[14] = 0.03;
  pose_cov.covariance[21] = 0.004;
  pose_cov.covariance[28] = 0.005;
  pose_cov.covariance[35] = 0.006;

  auto meas_cov = simple_ekf::poseWithCovarianceToEkfMeasurementCovariance(pose_cov);

  EXPECT_DOUBLE_EQ(meas_cov.data[ekf::PoseMeasurementCovariance::X],     0.01);
  EXPECT_DOUBLE_EQ(meas_cov.data[ekf::PoseMeasurementCovariance::Y],     0.02);
  EXPECT_DOUBLE_EQ(meas_cov.data[ekf::PoseMeasurementCovariance::Z],     0.03);
  EXPECT_DOUBLE_EQ(meas_cov.data[ekf::PoseMeasurementCovariance::ROLL],  0.004);
  EXPECT_DOUBLE_EQ(meas_cov.data[ekf::PoseMeasurementCovariance::PITCH], 0.005);
  EXPECT_DOUBLE_EQ(meas_cov.data[ekf::PoseMeasurementCovariance::YAW],   0.006);
}

// ---------------------------------------------------------------------------
// Angle unwrapping
// ---------------------------------------------------------------------------

// No wrapping needed: measurement is close to state yaw
TEST(UtilsAngleUnwrapTest, NoBoundary)
{
  ekf::State state;
  state.data.fill(0.0);
  state.data[ekf::State::YAW] = 0.0;

  double measured_yaw = 0.1;
  double qz = std::sin(measured_yaw / 2.0);
  double qw = std::cos(measured_yaw / 2.0);
  auto msg = makePoseMsg("map", 0.0, 0.0, 0.0, 0.0, 0.0, qz, qw);

  auto raw = simple_ekf::poseWithCovarianceToRawEkfMeasurement(msg);
  auto meas = simple_ekf::unwrapPoseMeasurement(raw, state);
  EXPECT_NEAR(meas.data[ekf::PoseMeasurement::YAW], 0.1, 1e-9);
}

// State yaw = 3.0 rad (EKF has tracked past +π); sensor yaw = -3.1 rad
// (getRPY wraps it into [-π, π]). Without unwrapping this looks like a 6.1 rad
// jump. The correct unwrapped value is ≈ 3.18 rad.
TEST(UtilsAngleUnwrapTest, PositiveSideToNegative)
{
  ekf::State state;
  state.data.fill(0.0);
  state.data[ekf::State::YAW] = 3.0;

  // Encode angle -3.1 as quaternion; getRPY will return -3.1
  double measured_yaw = -3.1;
  double qz = std::sin(measured_yaw / 2.0);
  double qw = std::cos(measured_yaw / 2.0);
  auto msg = makePoseMsg("map", 0.0, 0.0, 0.0, 0.0, 0.0, qz, qw);

  auto raw = simple_ekf::poseWithCovarianceToRawEkfMeasurement(msg);
  auto meas = simple_ekf::unwrapPoseMeasurement(raw, state);

  // diff = -3.1 - 3.0 = -6.1; round(-6.1/2π) = -1; corrected diff = -6.1 + 2π ≈ 0.18
  // result = 3.0 + 0.18 ≈ 3.18
  double expected = 3.0 + ((-3.1 - 3.0) - 2.0 * M_PI * std::round((-3.1 - 3.0) / (2.0 * M_PI)));
  EXPECT_NEAR(meas.data[ekf::PoseMeasurement::YAW], expected, 1e-9);
  // The result should NOT be -3.1 (the raw sensor value)
  EXPECT_FALSE(std::abs(meas.data[ekf::PoseMeasurement::YAW] - (-3.1)) < 0.01);
}

// State yaw = -3.0; sensor yaw = +3.1 → unwrapped ≈ -3.18
TEST(UtilsAngleUnwrapTest, NegativeSideToPositive)
{
  ekf::State state;
  state.data.fill(0.0);
  state.data[ekf::State::YAW] = -3.0;

  double measured_yaw = 3.1;
  double qz = std::sin(measured_yaw / 2.0);
  double qw = std::cos(measured_yaw / 2.0);
  auto msg = makePoseMsg("map", 0.0, 0.0, 0.0, 0.0, 0.0, qz, qw);

  auto raw = simple_ekf::poseWithCovarianceToRawEkfMeasurement(msg);
  auto meas = simple_ekf::unwrapPoseMeasurement(raw, state);

  double expected = -3.0 + ((3.1 - (-3.0)) - 2.0 * M_PI * std::round((3.1 - (-3.0)) / (2.0 * M_PI)));
  EXPECT_NEAR(meas.data[ekf::PoseMeasurement::YAW], expected, 1e-9);
}

// ---------------------------------------------------------------------------
// transformPoseToMapFrame tests
// ---------------------------------------------------------------------------

// A pose already expressed in a "*/map" frame passes through unchanged.
TEST(UtilsTransformFrameTest, MapFrame_PoseUnchanged)
{
  StateTransforms transforms;  // identity
  tf2::Transform earth_to_map;
  earth_to_map.setIdentity();

  auto msg = makePoseMsg("drone0/map", 2.0, 3.0, 1.0);
  auto result = simple_ekf::transformPoseToMapFrame(transforms, earth_to_map, msg);

  EXPECT_NEAR(result.pose.pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(result.pose.pose.position.y, 3.0, 1e-9);
  EXPECT_NEAR(result.pose.pose.position.z, 1.0, 1e-9);
}

// A pose in the "earth" frame is converted to "map" via
// map_pose = earth_to_map^-1 * earth_pose.
TEST(UtilsTransformFrameTest, EarthFrame_UsesEarthToMap)
{
  StateTransforms transforms;  // identity
  // earth is 10 m from map origin: earth_to_map = translate(10,0,0)
  tf2::Transform earth_to_map = makeTranslation(10.0, 0.0, 0.0);

  // Pose in earth frame at (10,0,0) → in map frame should be at (0,0,0)
  // map_pose = earth_to_map^-1 * earth_pose
  auto msg = makePoseMsg("earth", 10.0, 0.0, 0.0);
  auto result = simple_ekf::transformPoseToMapFrame(transforms, earth_to_map, msg);

  EXPECT_NEAR(result.pose.pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(result.pose.pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(result.pose.pose.position.z, 0.0, 1e-9);
}

// A pose in the "odom" frame is converted to "map" via
// map_pose = map_to_odom * odom_pose.
TEST(UtilsTransformFrameTest, OdomFrame_UsesMapToOdom)
{
  // map origin is 5 m from odom origin: map_to_odom = translate(5,0,0)
  tf2::Transform map_to_odom = makeTranslation(5.0, 0.0, 0.0);
  tf2::Transform odom_to_base;
  odom_to_base.setIdentity();
  tf2::Transform map_to_base = map_to_odom;

  StateTransforms transforms(map_to_base, map_to_odom, odom_to_base);
  tf2::Transform earth_to_map;
  earth_to_map.setIdentity();

  // Pose in odom at (1,0,0) → map_pose = map_to_odom * odom_pose = (6,0,0)
  auto msg = makePoseMsg("odom", 1.0, 0.0, 0.0);
  auto result = simple_ekf::transformPoseToMapFrame(transforms, earth_to_map, msg);

  EXPECT_NEAR(result.pose.pose.position.x, 6.0, 1e-9);
  EXPECT_NEAR(result.pose.pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(result.pose.pose.position.z, 0.0, 1e-9);
}

// A pose in the "base_link" frame is converted to "map" via
// map_pose = map_to_base * base_pose.
TEST(UtilsTransformFrameTest, BaseFrame_UsesMapToBase)
{
  // Drone is at (3,0,0) in map
  tf2::Transform map_to_base = makeTranslation(3.0, 0.0, 0.0);
  tf2::Transform map_to_odom;
  map_to_odom.setIdentity();
  tf2::Transform odom_to_base = map_to_base;

  StateTransforms transforms(map_to_base, map_to_odom, odom_to_base);
  tf2::Transform earth_to_map;
  earth_to_map.setIdentity();

  // Pose at base_link origin (0,0,0) in base frame → (3,0,0) in map
  auto msg = makePoseMsg("base_link", 0.0, 0.0, 0.0);
  auto result = simple_ekf::transformPoseToMapFrame(transforms, earth_to_map, msg);

  EXPECT_NEAR(result.pose.pose.position.x, 3.0, 1e-9);
}

// After rotating the covariance by 90° yaw, x-var and y-var should swap.
TEST(UtilsTransformFrameTest, CovarianceRotated_90DegYaw)
{
  StateTransforms transforms;
  // earth_to_map = 90° yaw rotation
  tf2::Transform earth_to_map = makeYawRotation(M_PI / 2.0);

  // Input pose in earth frame with diagonal pos covariance [1, 4, 9]
  auto msg = makePoseMsg("earth", 0.0, 0.0, 0.0);
  msg.pose.covariance = makeDiagCov36(1.0, 4.0, 9.0, 1.0, 1.0, 1.0);

  auto result = simple_ekf::transformPoseToMapFrame(transforms, earth_to_map, msg);

  // 90° yaw rotation swaps x↔y variances: Cov_out = R * Cov_in * R^T
  // R (90° yaw): [[0,-1,0],[1,0,0],[0,0,1]]
  // [1,0; 0,4] rotated by 90° → [4,0; 0,1]
  EXPECT_NEAR(result.pose.covariance[0], 4.0, 1e-9);  // x variance becomes 4
  EXPECT_NEAR(result.pose.covariance[7], 1.0, 1e-9);  // y variance becomes 1
  EXPECT_NEAR(result.pose.covariance[14], 9.0, 1e-9); // z variance unchanged
}

// ---------------------------------------------------------------------------
// ekfStateToTwist tests
// ---------------------------------------------------------------------------

// With map_to_base = identity, the state velocity passes straight through
// to the base-frame linear twist, and angular velocity equals the raw IMU
// reading when gyro bias is zero.
TEST(UtilsTwistTest, VelocityInBaseFrame_Identity)
{
  ekf::State state;
  state.data.fill(0.0);
  state.data[ekf::State::VX] = 1.0;

  tf2::Transform map_to_base;
  map_to_base.setIdentity();

  sensor_msgs::msg::Imu imu;
  imu.angular_velocity.x = 0.1;
  imu.angular_velocity.y = 0.2;
  imu.angular_velocity.z = 0.3;

  auto twist = simple_ekf::ekfStateToTwist(state, map_to_base, imu);

  EXPECT_NEAR(twist.twist.linear.x, 1.0, 1e-12);
  EXPECT_NEAR(twist.twist.linear.y, 0.0, 1e-12);
  EXPECT_NEAR(twist.twist.linear.z, 0.0, 1e-12);
  EXPECT_NEAR(twist.twist.angular.x, 0.1, 1e-12);  // no bias → IMU value
  EXPECT_NEAR(twist.twist.angular.y, 0.2, 1e-12);
  EXPECT_NEAR(twist.twist.angular.z, 0.3, 1e-12);
}

// The output angular velocity is the IMU measurement minus the EKF's
// estimated gyroscope bias, per axis.
TEST(UtilsTwistTest, GyroBiasSubtracted)
{
  ekf::State state;
  state.data.fill(0.0);
  state.data[ekf::State::WBX] = 0.05;
  state.data[ekf::State::WBY] = 0.05;
  state.data[ekf::State::WBZ] = 0.05;

  tf2::Transform map_to_base;
  map_to_base.setIdentity();

  sensor_msgs::msg::Imu imu;
  imu.angular_velocity.x = 0.1;
  imu.angular_velocity.y = 0.2;
  imu.angular_velocity.z = 0.3;

  auto twist = simple_ekf::ekfStateToTwist(state, map_to_base, imu);

  EXPECT_NEAR(twist.twist.angular.x, 0.05, 1e-12);  // 0.1 - 0.05
  EXPECT_NEAR(twist.twist.angular.y, 0.15, 1e-12);  // 0.2 - 0.05
  EXPECT_NEAR(twist.twist.angular.z, 0.25, 1e-12);  // 0.3 - 0.05
}

// ---------------------------------------------------------------------------
// StateTransforms tests
// ---------------------------------------------------------------------------

// Building StateTransforms from an EKF state derives map_to_base from the
// state's position, and with map_to_odom = identity, odom_to_base equals
// map_to_base.
TEST(UtilsStateTransformsTest, ConstructFromEkfState_OdomToBaseConsistency)
{
  // EKF state at (1,2,3), identity orientation
  ekf::State state;
  state.data.fill(0.0);
  state.data[ekf::State::X] = 1.0;
  state.data[ekf::State::Y] = 2.0;
  state.data[ekf::State::Z] = 3.0;

  // With identity map_to_odom, odom_to_base should equal map_to_base
  tf2::Transform identity;
  identity.setIdentity();
  StateTransforms transforms(state, identity);

  EXPECT_NEAR(transforms.map_to_base.getOrigin().x(), 1.0, 1e-9);
  EXPECT_NEAR(transforms.map_to_base.getOrigin().y(), 2.0, 1e-9);
  EXPECT_NEAR(transforms.map_to_base.getOrigin().z(), 3.0, 1e-9);

  // odom_to_base = identity^-1 * map_to_base = map_to_base
  EXPECT_NEAR(transforms.odom_to_base.getOrigin().x(), 1.0, 1e-9);
  EXPECT_NEAR(transforms.odom_to_base.getOrigin().y(), 2.0, 1e-9);
  EXPECT_NEAR(transforms.odom_to_base.getOrigin().z(), 3.0, 1e-9);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
