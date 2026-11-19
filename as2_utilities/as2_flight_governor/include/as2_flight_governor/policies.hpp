// Copyright 2026 Vishwa
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
//    * Neither the name of the copyright holder nor the names of its
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
* @file policies.hpp
*
* Built-in flight governor policies.
*
* @author Vishwa
*/

#ifndef AS2_FLIGHT_GOVERNOR__POLICIES_HPP_
#define AS2_FLIGHT_GOVERNOR__POLICIES_HPP_

#include <string>

#include "as2_flight_governor/governor_core.hpp"

namespace as2_flight_governor
{

/// What a policy does when a command breaks it.
enum class Action
{
  CLAMP,  ///< Modify the command so that it complies.
  VETO,   ///< Reject the command and hold.
};

/**
 * Axis-aligned box the vehicle must stay inside.
 *
 * - Position setpoints outside the box are moved to the nearest point inside.
 * - Velocity components are limited so that the vehicle, at its current
 *   position, cannot cross a face of the box within `lookahead` seconds.
 *   Outside the box, motion back towards it is still allowed.
 */
class GeofenceBoxPolicy : public Policy
{
public:
  GeofenceBoxPolicy(
    std::string name, const Vec3 & min, const Vec3 & max,
    double lookahead = 1.0, Action action = Action::CLAMP);

  std::string type() const override {return "geofence_box";}
  PolicyResult apply(const Command & command, const State & state) const override;

private:
  Vec3 min_;
  Vec3 max_;
  double lookahead_;
  Action action_;
};

/**
 * Speed limits on velocity setpoints. Horizontal speed is limited as a vector
 * (direction is kept), vertical speed is limited per sign. A negative limit
 * disables that axis.
 */
class MaxSpeedPolicy : public Policy
{
public:
  MaxSpeedPolicy(std::string name, double horizontal, double vertical);

  std::string type() const override {return "max_speed";}
  PolicyResult apply(const Command & command, const State & state) const override;

private:
  double horizontal_;
  double vertical_;
};

/**
 * Sphere the vehicle must stay out of, with an extra safety margin.
 *
 * - Position setpoints inside `radius + margin` are projected onto its surface.
 * - The velocity component towards the centre is limited so the vehicle cannot
 *   enter the sphere within `lookahead` seconds. Tangential motion is kept, so
 *   the vehicle can slide around the obstacle.
 */
class KeepoutSpherePolicy : public Policy
{
public:
  KeepoutSpherePolicy(
    std::string name, const Vec3 & center, double radius, double margin = 0.0,
    double lookahead = 1.0, Action action = Action::CLAMP);

  std::string type() const override {return "keepout_sphere";}
  PolicyResult apply(const Command & command, const State & state) const override;

private:
  Vec3 center_;
  double radius_;
  double margin_;
  double lookahead_;
  Action action_;
};

/**
 * Lower speed limit while the vehicle is inside a spherical zone, e.g. near a
 * landing pad or people. Limits the norm of the velocity setpoint.
 */
class SpeedZonePolicy : public Policy
{
public:
  SpeedZonePolicy(std::string name, const Vec3 & center, double radius, double max_speed);

  std::string type() const override {return "speed_zone";}
  PolicyResult apply(const Command & command, const State & state) const override;

private:
  Vec3 center_;
  double radius_;
  double max_speed_;
};

}  // namespace as2_flight_governor

#endif  // AS2_FLIGHT_GOVERNOR__POLICIES_HPP_
