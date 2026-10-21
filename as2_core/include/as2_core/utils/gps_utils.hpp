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

/********************************************************************************************
 *  \file       frame_utils.hpp
 *  \brief      Aerostack2 frame utils header file.
 *  \authors    Rafael Pérez Seguí
 *              Pedro Arias Pérez
 ********************************************************************************/

#ifndef AS2_CORE__UTILS__GPS_UTILS_HPP_
#define AS2_CORE__UTILS__GPS_UTILS_HPP_

#include <string>
#include <GeographicLib/LocalCartesian.hpp>

#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace as2
{
namespace gps
{

static const GeographicLib::Geocentric & earth = GeographicLib::Geocentric::WGS84();
static const char default_global_frame[] = "earth";  // wgs84 --> ROS REP105 Name Convention

class OriginNonSet : public std::runtime_error
{
public:
  OriginNonSet()
  : std::runtime_error("origin is not set") {}
};

class OriginAlreadySet : public std::runtime_error
{
public:
  OriginAlreadySet()
  : std::runtime_error("origin can only be set once") {}
};

class GpsHandler : private GeographicLib::LocalCartesian
{
public:
  /**
   * @brief Construct a new Gps Handler object based on WGS84 ellipsoid
   *
   */
  GpsHandler()
  : GeographicLib::LocalCartesian(earth) {}

  /**
   * @brief Construct a new Gps Handler object based on WGS84 ellipsoid with a given origin
   *
   * @param lat0 Latitude at origin (degrees)
   * @param lon0 Longitude at origin (degrees)
   * @param h0 Altitude at origin (meters)
   */
  GpsHandler(double lat0, double lon0, double h0 = 0)
  : GeographicLib::LocalCartesian(lat0, lon0, h0, earth)
  {
    this->is_origin_set_ = true;
  }

  /**
   * @brief Set the frame id stamped on the messages this handler produces.
   *
   * Defaults to "earth". A node that renames its global frame passes
   * as2::Node::getEarthFrameId() here.
   *
   * @param global_frame Frame id of the global frame.
   */
  void setGlobalFrame(const std::string & global_frame) {global_frame_ = global_frame;}

  /**
   * @brief Set the frame id stamped on the local-coordinate messages.
   *
   * Defaults to "map". A node that renames its map frame passes
   * as2::Node::getMapFrameId() here.
   *
   * @param local_frame Frame id of the map frame.
   */
  void setLocalFrame(const std::string & local_frame) {local_frame_ = local_frame;}

  /**
   * @brief Get the frame id stamped on the local-coordinate messages.
   *
   * @return Map frame id.
   */
  const std::string & getLocalFrame() const {return local_frame_;}

  /**
   * @brief Get the frame id stamped on the messages this handler produces.
   *
   * @return Global frame id.
   */
  const std::string & getGlobalFrame() const {return global_frame_;}

  /****************************************************************************************
   *                                                                                      *
   *                                    ORIGIN                                            *
   *                                                                                      *
   ***************************************************************************************/
  void setOrigin(const double & lat0, const double & lon0, const double & h0 = 0);
  void setOrigin(const sensor_msgs::msg::NavSatFix & fix);
  void setOrigin(const geographic_msgs::msg::GeoPoseStamped & gps);
  void getOrigin(double & rLat, double & rLon, double & rH);
  void getOrigin(geographic_msgs::msg::GeoPoseStamped & gps);

  /****************************************************************************************
   *                                                                                      *
   *                        Geodesic LLA to Local Cartesian                               *
   *                                                                                      *
   ***************************************************************************************/
  void LatLon2Local(
    const double & lat, const double & lon, const double & h, double & rX, double & rY,
    double & rZ);
  void LatLon2Local(const sensor_msgs::msg::NavSatFix & fix, double & rX, double & rY, double & rZ);
  void LatLon2Local(
    const geographic_msgs::msg::GeoPoseStamped & gps, double & rX, double & rY, double & rZ);
  void LatLon2Local(
    const double & lat, const double & lon, const double & h, geometry_msgs::msg::PoseStamped & ps);
  void LatLon2Local(const sensor_msgs::msg::NavSatFix & fix, geometry_msgs::msg::PoseStamped & ps);
  void LatLon2Local(
    const geographic_msgs::msg::GeoPoseStamped & gps, geometry_msgs::msg::PoseStamped & ps);

  /****************************************************************************************
   *                                                                                      *
   *                        Local cartesian to Geodesic LLA                               *
   *                                                                                      *
   ***************************************************************************************/
  void Local2LatLon(
    const double & x, const double & y, const double & z, double & rLat, double & rLon,
    double & rH);
  void Local2LatLon(
    const double & x, const double & y, const double & z,
    geographic_msgs::msg::GeoPoseStamped & gps);
  void Local2LatLon(
    const geometry_msgs::msg::PoseStamped & ps, double & rLat, double & rLon, double & rH);
  void Local2LatLon(
    const geometry_msgs::msg::PoseStamped & ps, geographic_msgs::msg::GeoPoseStamped & gps);

  /****************************************************************************************
   *                                                                                      *
   *                      Geodesic LLA to Earth-Centered-Earth-Fixed                      *
   *                                                                                      *
   ***************************************************************************************/
  static void LatLon2Ecef(
    const double & lat, const double & lon, const double & h, double & rX, double & rY,
    double & rZ);
  static void LatLon2Ecef(
    const sensor_msgs::msg::NavSatFix & fix, double & rX, double & rY, double & rZ);
  static void LatLon2Ecef(
    const geographic_msgs::msg::GeoPoseStamped & gps, double & rX, double & rY, double & rZ);
  static void LatLon2Ecef(
    const double & lat, const double & lon, const double & h, geometry_msgs::msg::PoseStamped & ps);
  static void LatLon2Ecef(
    const sensor_msgs::msg::NavSatFix & fix, geometry_msgs::msg::PoseStamped & ps);
  static void LatLon2Ecef(
    const geographic_msgs::msg::GeoPoseStamped & gps, geometry_msgs::msg::PoseStamped & ps);

  /****************************************************************************************
   *                                                                                      *
   *                      Earth-Centered-Earth-Fixed to Geodesic LLA                      *
   *                                                                                      *
   ***************************************************************************************/
  static void Ecef2LatLon(
    const double & x, const double & y, const double & z, double & rLat, double & rLon,
    double & rH);
  static void Ecef2LatLon(
    const double & x, const double & y, const double & z,
    geographic_msgs::msg::GeoPoseStamped & gps);
  static void Ecef2LatLon(
    const geometry_msgs::msg::PoseStamped & ps, double & rLat, double & rLon, double & rH);
  static void Ecef2LatLon(
    const geometry_msgs::msg::PoseStamped & ps, geographic_msgs::msg::GeoPoseStamped & gps);

private:
  std::string global_frame_ = default_global_frame;
  std::string local_frame_ = "map";  // local world fixed --> ROS REP105 Name Convention
  bool is_origin_set_ = false;
};  // GpsHandler

}  // namespace gps
}  // namespace as2

#endif  // AS2_CORE__UTILS__GPS_UTILS_HPP_
