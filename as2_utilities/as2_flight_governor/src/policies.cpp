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
* @file policies.cpp
*
* Built-in flight governor policies.
*
* @author Vishwa
*/

#include "as2_flight_governor/policies.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace as2_flight_governor
{

namespace
{

constexpr double kEpsilon = 1e-9;

double & axis(Vec3 & v, int i)
{
  return i == 0 ? v.x : (i == 1 ? v.y : v.z);
}

double axis(const Vec3 & v, int i)
{
  return i == 0 ? v.x : (i == 1 ? v.y : v.z);
}

bool isFinite(const Vec3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

PolicyResult allow(const Command & command)
{
  return PolicyResult{Verdict::ALLOW, command, ""};
}

PolicyResult violation(Action action, const Command & clamped, const std::string & reason)
{
  return PolicyResult{action == Action::VETO ? Verdict::VETO : Verdict::CLAMP, clamped, reason};
}

void requirePositive(double value, const std::string & what, const std::string & name)
{
  if (!(value > 0.0) || !std::isfinite(value)) {
    throw std::invalid_argument(name + ": " + what + " must be a positive number");
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// GeofenceBoxPolicy

GeofenceBoxPolicy::GeofenceBoxPolicy(
  std::string name, const Vec3 & min, const Vec3 & max, double lookahead, Action action)
: Policy(std::move(name)), min_(min), max_(max), lookahead_(lookahead), action_(action)
{
  if (!isFinite(min_) || !isFinite(max_)) {
    throw std::invalid_argument(this->name() + ": min and max must be finite");
  }
  for (int i = 0; i < 3; ++i) {
    if (!(axis(min_, i) < axis(max_, i))) {
      throw std::invalid_argument(this->name() + ": min must be smaller than max on every axis");
    }
  }
  requirePositive(lookahead_, "lookahead", this->name());
}

PolicyResult GeofenceBoxPolicy::apply(const Command & command, const State & state) const
{
  Command out = command;
  bool violated = false;

  if (out.hasPosition()) {
    for (int i = 0; i < 3; ++i) {
      const double clamped = std::clamp(axis(out.position, i), axis(min_, i), axis(max_, i));
      if (clamped != axis(out.position, i)) {
        axis(out.position, i) = clamped;
        violated = true;
      }
    }
  }

  if (out.hasVelocity()) {
    for (int i = 0; i < 3; ++i) {
      const double p = axis(state.position, i);
      double & v = axis(out.velocity, i);
      // Fastest speed towards each face that cannot reach it within lookahead.
      const double max_up = std::max(0.0, (axis(max_, i) - p) / lookahead_);
      const double max_down = std::min(0.0, (axis(min_, i) - p) / lookahead_);
      const double clamped = std::clamp(v, max_down, max_up);
      if (clamped != v) {
        v = clamped;
        violated = true;
      }
    }
  }

  if (!violated) {
    return allow(command);
  }
  return violation(action_, out, "command would leave the geofence box");
}

// ---------------------------------------------------------------------------
// MaxSpeedPolicy

MaxSpeedPolicy::MaxSpeedPolicy(std::string name, double horizontal, double vertical)
: Policy(std::move(name)), horizontal_(horizontal), vertical_(vertical)
{
  if (!std::isfinite(horizontal_) || !std::isfinite(vertical_)) {
    throw std::invalid_argument(this->name() + ": limits must be finite");
  }
  if (horizontal_ < 0.0 && vertical_ < 0.0) {
    throw std::invalid_argument(this->name() + ": at least one limit must be enabled (>= 0)");
  }
}

PolicyResult MaxSpeedPolicy::apply(const Command & command, const State &) const
{
  if (!command.hasVelocity() && !command.isSpeedLimit()) {
    return allow(command);
  }
  Command out = command;
  Vec3 & v = out.velocity;
  bool violated = false;

  if (command.isSpeedLimit()) {
    // Per-axis limits: an unlimited axis takes the cap, then the horizontal
    // pair is scaled so that even a diagonal move stays under the cap.
    if (horizontal_ >= 0.0) {
      if (std::isinf(v.x)) {v.x = horizontal_;}
      if (std::isinf(v.y)) {v.y = horizontal_;}
      const double h = std::hypot(v.x, v.y);
      if (h > horizontal_) {
        v.x *= horizontal_ / h;
        v.y *= horizontal_ / h;
      }
    }
    if (vertical_ >= 0.0) {
      v.z = std::min(v.z, vertical_);
    }
    violated = v.x != command.velocity.x || v.y != command.velocity.y ||
      v.z != command.velocity.z;
  } else {
    if (horizontal_ >= 0.0) {
      const double h = std::hypot(v.x, v.y);
      if (h > horizontal_) {
        const double scale = h > kEpsilon ? horizontal_ / h : 0.0;
        v.x *= scale;
        v.y *= scale;
        violated = true;
      }
    }
    if (vertical_ >= 0.0 && std::abs(v.z) > vertical_) {
      v.z = std::copysign(vertical_, v.z);
      violated = true;
    }
  }

  if (!violated) {
    return allow(command);
  }
  return PolicyResult{Verdict::CLAMP, out, "speed limited"};
}

// ---------------------------------------------------------------------------
// KeepoutSpherePolicy

KeepoutSpherePolicy::KeepoutSpherePolicy(
  std::string name, const Vec3 & center, double radius, double margin, double lookahead,
  Action action)
: Policy(std::move(name)), center_(center), radius_(radius), margin_(margin),
  lookahead_(lookahead), action_(action)
{
  if (!isFinite(center_)) {
    throw std::invalid_argument(this->name() + ": center must be finite");
  }
  requirePositive(radius_, "radius", this->name());
  if (!(margin_ >= 0.0) || !std::isfinite(margin_)) {
    throw std::invalid_argument(this->name() + ": margin must be zero or positive");
  }
  requirePositive(lookahead_, "lookahead", this->name());
}

PolicyResult KeepoutSpherePolicy::apply(const Command & command, const State & state) const
{
  const double limit = radius_ + margin_;
  Command out = command;
  bool violated = false;

  if (out.hasPosition()) {
    const Vec3 offset = out.position - center_;
    const double dist = norm(offset);
    if (dist < limit) {
      // Push the target out along the centre-to-target ray. If the target is
      // the centre itself, use the centre-to-vehicle ray, then straight up.
      Vec3 dir = offset;
      if (dist < kEpsilon) {
        dir = state.position - center_;
      }
      if (norm(dir) < kEpsilon) {
        dir = Vec3{0.0, 0.0, 1.0};
      }
      out.position = center_ + dir * (limit / norm(dir));
      violated = true;
    }
  }

  if (out.hasVelocity()) {
    const Vec3 offset = state.position - center_;
    const double dist = norm(offset);
    if (dist < kEpsilon) {
      // No defined direction out of the sphere: stop.
      if (norm(out.velocity) > 0.0) {
        out.velocity = Vec3{};
        violated = true;
      }
    } else {
      const Vec3 outward = offset * (1.0 / dist);
      const double inward_speed = -dot(out.velocity, outward);
      const double allowed_inward = std::max(0.0, (dist - limit) / lookahead_);
      if (inward_speed > allowed_inward) {
        out.velocity = out.velocity + outward * (inward_speed - allowed_inward);
        violated = true;
      }
    }
  }

  if (!violated) {
    return allow(command);
  }
  return violation(action_, out, "command would enter the keep-out sphere");
}

// ---------------------------------------------------------------------------
// SpeedZonePolicy

SpeedZonePolicy::SpeedZonePolicy(
  std::string name, const Vec3 & center, double radius, double max_speed)
: Policy(std::move(name)), center_(center), radius_(radius), max_speed_(max_speed)
{
  if (!isFinite(center_)) {
    throw std::invalid_argument(this->name() + ": center must be finite");
  }
  requirePositive(radius_, "radius", this->name());
  if (!(max_speed_ >= 0.0) || !std::isfinite(max_speed_)) {
    throw std::invalid_argument(this->name() + ": max_speed must be zero or positive");
  }
}

PolicyResult SpeedZonePolicy::apply(const Command & command, const State & state) const
{
  if ((!command.hasVelocity() && !command.isSpeedLimit()) ||
    norm(state.position - center_) > radius_)
  {
    return allow(command);
  }
  Command out = command;
  Vec3 & v = out.velocity;
  if (command.isSpeedLimit()) {
    if (std::isinf(v.x)) {v.x = max_speed_;}
    if (std::isinf(v.y)) {v.y = max_speed_;}
    if (std::isinf(v.z)) {v.z = max_speed_;}
  }
  const double speed = norm(v);
  if (speed > max_speed_) {
    v = v * (speed > kEpsilon ? max_speed_ / speed : 0.0);
  }
  if (v.x == command.velocity.x && v.y == command.velocity.y && v.z == command.velocity.z) {
    return allow(command);
  }
  return PolicyResult{Verdict::CLAMP, out, "speed limited inside zone"};
}

}  // namespace as2_flight_governor
