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
* @file external_localization.hpp
*
* State estimation plugin that publishes estimates computed outside aerostack2: a drifting
* odometry, plus an optional drift-free localization pose that corrects it through map to odom.
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*          Rodrigo Da Silva Gómez
*/

#ifndef EXTERNAL_LOCALIZATION__EXTERNAL_LOCALIZATION_HPP_
#define EXTERNAL_LOCALIZATION__EXTERNAL_LOCALIZATION_HPP_

#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

#include <as2_core/utils/tf_utils.hpp>
#include <as2_core/names/services.hpp>
#include <as2_core/utils/gps_utils.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include <as2_msgs/srv/set_origin.hpp>

#include "as2_state_estimator/plugin_base.hpp"
#include "as2_core/names/topics.hpp"

namespace external_localization
{

/**
 * @brief Interpolate between two transforms, linearly in translation and by slerp in rotation.
 *
 * @param from Transform at ratio 0.
 * @param to Transform at ratio 1.
 * @param ratio Interpolation ratio, in [0, 1].
 * @return The interpolated transform.
 */
tf2::Transform interpolate(const tf2::Transform & from, const tf2::Transform & to, double ratio);

/**
 * @brief Move a transform toward a target, by at most a given translation and rotation.
 *
 * @param from Current transform.
 * @param to Target transform.
 * @param max_translation Largest translation step allowed (m).
 * @param max_rotation Largest rotation step allowed (rad).
 * @return The transform after the step, equal to the target when it is within both limits.
 */
tf2::Transform stepToward(
  const tf2::Transform & from, const tf2::Transform & to,
  double max_translation, double max_rotation);

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
protected:
  // Parameter namespace and odometry topic key. The raw_odometry alias overrides both so its
  // existing configs keep working.
  std::string prefix_ = "external_localization";
  std::string odometry_topic_key_ = "odometry.topic";

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  bool set_earth_map_manually_ = false;
  bool earth_to_map_set_ = false;
  bool map_to_odom_set_ = false;
  tf2::Transform earth_to_map_ = tf2::Transform::getIdentity();

  // GPS-based earth->map support
  bool use_gps_ = false;

  // Where the geodetic datum comes from. FIRST_GPS adopts the first fix, MANUAL takes it from
  // parameters, SERVICE holds the first fix aside and waits for set_origin to supply it.
  enum class OriginSource { FIRST_GPS, MANUAL, SERVICE };
  OriginSource origin_source_ = OriginSource::FIRST_GPS;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Service<as2_msgs::srv::SetOrigin>::SharedPtr set_origin_srv_;
  rclcpp::Service<as2_msgs::srv::GetOrigin>::SharedPtr get_origin_srv_;

  geographic_msgs::msg::GeoPoint::UniquePtr origin_;
  sensor_msgs::msg::NavSatFix::UniquePtr gps_pose_;

  // Localization input. Without it map to odom stays a static identity.
  enum class JumpMode { NONE, CLAMP, SMOOTH };

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    localization_sub_;
  std::string localization_topic_;
  bool use_localization_ = false;
  bool wait_for_localization_ = true;
  bool localized_ = false;
  double odometry_history_ = 1.0;
  double localization_timeout_ = 1.0;
  JumpMode jump_mode_ = JumpMode::NONE;
  double max_rate_translation_ = 0.5;
  double max_rate_rotation_ = 0.3;
  double smoothing_time_ = 1.0;
  double warn_translation_ = 1.0;

  std::deque<std::pair<rclcpp::Time, tf2::Transform>> odom_history_;
  tf2::Transform map_to_odom_ = tf2::Transform::getIdentity();
  tf2::Transform map_to_odom_target_ = tf2::Transform::getIdentity();
  // ROS clock type so differences with message stamps and now() do not throw
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_localization_time_{0, 0, RCL_ROS_TIME};

public:
  /**
   * @brief Construct the plugin. It is only usable after onSetup().
   */
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}

  /**
   * @brief Setup the plugin
   *
   * Reads the parameters and subscribes to the odometry topic, and to the localization topic
   * when one is set.
   */
  void onSetup() override;

  /**
   * @brief Get the list of transformation types provided by this plugin
   *
   * @return std::vector<as2_state_estimator::TransformInformatonType> List of available transformations
   */
  std::vector<as2_state_estimator::TransformInformatonType> getTransformationTypesAvailable() const
  override;

protected:
  /**
   * @brief Full parameter name, under this plugin's namespace.
   *
   * @param key Parameter name relative to the plugin namespace.
   * @return The namespaced parameter name.
   */
  std::string param(const std::string & key) const {return prefix_ + "." + key;}

private:
  /**
   * @brief Setup the TF tree by initializing earth-to-map and map-to-odom transforms
   */
  void setupTfTree();

  /**
   * @brief Setup GPS subscription and set_origin/get_origin services
   *
   * Used when use_gps_ is true to anchor the earth frame at a geodetic datum,
   * taken from the gps_origin parameters or from the first received fix. The
   * fix also places map, unless earth_map_transform pins it.
   */
  void setupGps();

  /**
   * @brief Read the localization parameters and subscribe to the localization topic.
   */
  void setupLocalization();

  /**
   * @brief Compute the earth-to-map transform from a GeoPoint origin and a GPS fix
   *
   * @param origin Geographic origin of the map frame
   * @param gps_pose GPS fix used to compute the local offset from the origin
   */
  void generateMapFrameFromGps(
    const geographic_msgs::msg::GeoPoint & origin,
    const sensor_msgs::msg::NavSatFix & gps_pose);

  /**
   * @brief Callback for GPS topic subscription
   *
   * Only the first received fix is used: it is stored, the origin is derived
   * from it if not already set (e.g. via the gps_origin parameters or the
   * set_origin service), and the earth-to-map transform is generated. The
   * subscription is dropped afterward.
   *
   * @param msg GPS fix message from topic
   */
  void gpsCallback(sensor_msgs::msg::NavSatFix::UniquePtr msg);

  /**
   * @brief Service callback to manually set the GPS origin
   *
   * @param request Contains the GeoPoint origin to set
   * @param response Whether the origin was set
   */
  void setOriginCallback(
    const as2_msgs::srv::SetOrigin::Request::SharedPtr request,
    as2_msgs::srv::SetOrigin::Response::SharedPtr response);

  /**
   * @brief Service callback to query the currently set GPS origin
   *
   * @param request Empty request
   * @param response Contains the GeoPoint origin, if set
   */
  void getOriginCallback(
    const as2_msgs::srv::GetOrigin::Request::SharedPtr request,
    as2_msgs::srv::GetOrigin::Response::SharedPtr response);

  /**
   * @brief Callback for odometry topic subscription
   *
   * Publishes odom to base_link and the twist. With localization enabled it also records the
   * odometry history, moves map to odom toward its target and publishes it.
   *
   * @param msg Odometry message from topic
   */
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Callback for the localization topic
   *
   * Computes the map to odom target from the pose and the odometry at the pose's timestamp.
   *
   * @param msg Pose of base_link in the map or earth frame
   */
  void localizationCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief Odom to base_link at a given time, interpolated from the odometry history.
   *
   * A time newer than the last odometry gets the last odometry.
   *
   * @param stamp Time to look up.
   * @return The transform, or nothing when the time is older than the history.
   */
  std::optional<tf2::Transform> odomToBaseAt(const rclcpp::Time & stamp) const;

  /**
   * @brief Move map to odom toward its target according to the jump mode.
   *
   * @param dt Time since the last odometry message (s).
   */
  void advanceMapToOdom(double dt);
};      // class Plugin
}       // namespace external_localization
#endif  // EXTERNAL_LOCALIZATION__EXTERNAL_LOCALIZATION_HPP_
