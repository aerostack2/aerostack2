/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef GZ_SIM_SYSTEMS_ODOMETRYPUBLISHER_HH_
#define GZ_SIM_SYSTEMS_ODOMETRYPUBLISHER_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class OdometryPublisherPrivate;

  /// \brief Odometry Publisher which can be attached to any entity in
  /// order to periodically publish 2D or 3D odometry data in the form of
  /// gz::msgs::Odometry messages.
  ///
  /// This version uses velocity integration to simulate realistic odometry
  /// drift, making it suitable for testing state estimation algorithms.
  ///
  /// ## System Parameters
  ///
  /// - `<odom_frame>`: Name of the world-fixed coordinate frame for the
  /// odometry message. This element is optional, and the default value
  /// is `{name_of_model}/odom`.
  ///
  /// - `<robot_base_frame>`: Name of the coordinate frame rigidly attached
  /// to the mobile robot base. This element is optional, and the default
  /// value is `{name_of_model}/base_footprint`.
  ///
  /// - `<odom_publish_frequency>`: Odometry publication frequency. This
  /// element is optional, and the default value is 50Hz.
  ///
  /// - `<odom_topic>`: Custom topic on which this system will publish odometry
  /// messages. This element is optional, and the default value is
  /// `/model/{name_of_model}/odometry`.
  ///
  /// - `<odom_covariance_topic>`: Custom topic on which this system will
  /// publish odometry with covariance messages. This element is optional, and
  /// the default value is `/model/{name_of_model}/odometry_with_covariance`.
  ///
  /// - `<tf_topic>`: Custom topic on which this system will publish the
  /// transform from `odom_frame` to `robot_base_frame`. This element is
  /// optional, and the default value is `/model/{name_of_model}/pose`.
  ///
  /// - `<dimensions>`: Number of dimensions to represent odometry. Only 2 and 3
  /// dimensional spaces are supported. This element is optional, and the
  /// default value is 2.
  ///
  /// - `<xyz_offset>`: Position offset relative to the body fixed frame, the
  /// default value is 0 0 0. This offset will be added to the odometry
  /// message.
  ///
  /// - `<rpy_offset>`: Rotation offset relative to the body fixed frame, the
  /// default value is 0 0 0. This offset will be added to the odometry
  /// message.
  ///
  /// - `<gaussian_noise>`: Standard deviation of the Gaussian noise to be added
  /// to both position and orientation. This element is optional, and the default
  /// value is 0. (Legacy parameter - use position_noise and angular_noise instead)
  ///
  /// - `<position_noise>`: Standard deviation of position noise in meters.
  /// This element is optional, and the default value is 0.
  ///
  /// - `<angular_noise>`: Standard deviation of angular noise in radians.
  /// This element is optional, and the default value is 0.
  ///
  /// ## Components
  ///
  /// This system uses the following components:
  ///
  /// - gz::sim::components::LinearVelocity: Linear velocity in body frame.
  /// Used to integrate position with noise.
  ///
  /// - gz::sim::components::AngularVelocity: Angular velocity in body frame.
  /// Used to integrate orientation with noise.
  ///
  /// - gz::sim::components::Pose: Initial pose for odometry starting point.
  ///
  /// ## Behavior
  ///
  /// Unlike ground-truth odometry, this plugin integrates velocities to compute
  /// pose, which introduces realistic drift over time. Noise is added to velocities
  /// using a random walk model (noise scales with 1/sqrt(dt)).
  ///
  /// Published velocities are in body frame (standard ROS convention), while
  /// pose is published in the odometry frame.
  class OdometryPublisher
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: OdometryPublisher();

    /// \brief Destructor
    public: ~OdometryPublisher() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<OdometryPublisherPrivate> dataPtr;
  };
}
}
}
}

#endif
