/*!*******************************************************************************************
 *  \file       frame_utils.hpp
 *  \brief      Aerostack2 frame utils header file.
 *  \authors    Rafael Pérez Seguí
 *              Pedro Arias Pérez
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

#ifndef GPS_UTILS_HPP_
#define GPS_UTILS_HPP_

#include <GeographicLib/LocalCartesian.hpp>
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace as2 {
namespace gps {

const static GeographicLib::Geocentric &earth = GeographicLib::Geocentric::WGS84();
const static std::string global_frame         = "earth";  // wgs84 --> ROS REP105 Name Convention

class OriginNonSet : public std::runtime_error {
public:
  OriginNonSet() : std::runtime_error("origin is not set") {}
};

class OriginAlreadySet : public std::runtime_error {
public:
  OriginAlreadySet() : std::runtime_error("origin can only be set once") {}
};

class GpsHandler : private GeographicLib::LocalCartesian {
public:
  /**
   * @brief Construct a new Gps Handler object based on WGS84 ellipsoid
   *
   */
  GpsHandler() : GeographicLib::LocalCartesian(earth){};

  /**
   * @brief Construct a new Gps Handler object based on WGS84 ellipsoid with a given origin
   *
   * @param lat0 Latitude at origin (degrees)
   * @param lon0 Longitude at origin (degrees)
   * @param h0 Altitude at origin (meters)
   */
  GpsHandler(double lat0, double lon0, double h0 = 0)
      : GeographicLib::LocalCartesian(lat0, lon0, h0, earth) {
    this->is_origin_set_ = true;
  };

  /****************************************************************************************
   *                                                                                      *
   *                                    ORIGIN                                            *
   *                                                                                      *
   ***************************************************************************************/
  void setOrigin(const double &lat0, const double &lon0, const double &h0 = 0);
  void setOrigin(const sensor_msgs::msg::NavSatFix &fix);
  void setOrigin(const geographic_msgs::msg::GeoPoseStamped &gps);
  void getOrigin(double &rLat, double &rLon, double &rH);
  void getOrigin(geographic_msgs::msg::GeoPoseStamped &gps);

  /****************************************************************************************
   *                                                                                      *
   *                        Geodesic LLA to Local Cartesian                               *
   *                                                                                      *
   ***************************************************************************************/
  void LatLon2Local(const double &lat,
                    const double &lon,
                    const double &h,
                    double &rX,
                    double &rY,
                    double &rZ);
  void LatLon2Local(const sensor_msgs::msg::NavSatFix &fix, double &rX, double &rY, double &rZ);
  void LatLon2Local(const geographic_msgs::msg::GeoPoseStamped &gps,
                    double &rX,
                    double &rY,
                    double &rZ);
  void LatLon2Local(const double &lat,
                    const double &lon,
                    const double &h,
                    geometry_msgs::msg::PoseStamped &ps);
  void LatLon2Local(const sensor_msgs::msg::NavSatFix &fix, geometry_msgs::msg::PoseStamped &ps);
  void LatLon2Local(const geographic_msgs::msg::GeoPoseStamped &gps,
                    geometry_msgs::msg::PoseStamped &ps);

  /****************************************************************************************
   *                                                                                      *
   *                        Local cartesian to Geodesic LLA                               *
   *                                                                                      *
   ***************************************************************************************/
  void Local2LatLon(const double &x,
                    const double &y,
                    const double &z,
                    double &rLat,
                    double &rLon,
                    double &rH);
  void Local2LatLon(const double &x,
                    const double &y,
                    const double &z,
                    geographic_msgs::msg::GeoPoseStamped &gps);
  void Local2LatLon(const geometry_msgs::msg::PoseStamped &ps,
                    double &rLat,
                    double &rLon,
                    double &rH);
  void Local2LatLon(const geometry_msgs::msg::PoseStamped &ps,
                    geographic_msgs::msg::GeoPoseStamped &gps);

  /****************************************************************************************
   *                                                                                      *
   *                      Geodesic LLA to Earth-Centered-Earth-Fixed                      *
   *                                                                                      *
   ***************************************************************************************/
  static void LatLon2Ecef(const double &lat,
                          const double &lon,
                          const double &h,
                          double &rX,
                          double &rY,
                          double &rZ);
  static void LatLon2Ecef(const sensor_msgs::msg::NavSatFix &fix,
                          double &rX,
                          double &rY,
                          double &rZ);
  static void LatLon2Ecef(const geographic_msgs::msg::GeoPoseStamped &gps,
                          double &rX,
                          double &rY,
                          double &rZ);
  static void LatLon2Ecef(const double &lat,
                          const double &lon,
                          const double &h,
                          geometry_msgs::msg::PoseStamped &ps);
  static void LatLon2Ecef(const sensor_msgs::msg::NavSatFix &fix,
                          geometry_msgs::msg::PoseStamped &ps);
  static void LatLon2Ecef(const geographic_msgs::msg::GeoPoseStamped &gps,
                          geometry_msgs::msg::PoseStamped &ps);

  /****************************************************************************************
   *                                                                                      *
   *                      Earth-Centered-Earth-Fixed to Geodesic LLA                      *
   *                                                                                      *
   ***************************************************************************************/
  static void Ecef2LatLon(const double &x,
                          const double &y,
                          const double &z,
                          double &rLat,
                          double &rLon,
                          double &rH);
  static void Ecef2LatLon(const double &x,
                          const double &y,
                          const double &z,
                          geographic_msgs::msg::GeoPoseStamped &gps);
  static void Ecef2LatLon(const geometry_msgs::msg::PoseStamped &ps,
                          double &rLat,
                          double &rLon,
                          double &rH);
  static void Ecef2LatLon(const geometry_msgs::msg::PoseStamped &ps,
                          geographic_msgs::msg::GeoPoseStamped &gps);

private:
  const std::string local_frame_ = "map";  // local world fixed --> ROS REP105 Name Convention
  bool is_origin_set_            = false;
};  // GpsHandler

}  // namespace gps
}  // namespace as2

#endif  // GPS_UTILS_HPP_
