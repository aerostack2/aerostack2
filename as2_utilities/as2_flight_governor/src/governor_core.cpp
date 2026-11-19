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
* @file governor_core.cpp
*
* Flight governor core: vector helpers and the policy-composing governor.
*
* @author Vishwa
*/

#include "as2_flight_governor/governor_core.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace as2_flight_governor
{

Vec3 operator+(const Vec3 & a, const Vec3 & b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
Vec3 operator-(const Vec3 & a, const Vec3 & b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
Vec3 operator*(const Vec3 & a, double s) {return {a.x * s, a.y * s, a.z * s};}
double dot(const Vec3 & a, const Vec3 & b) {return a.x * b.x + a.y * b.y + a.z * b.z;}
double norm(const Vec3 & a) {return std::sqrt(dot(a, a));}

namespace
{

bool isFinite(const Vec3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

}  // namespace

std::string toString(Verdict verdict)
{
  switch (verdict) {
    case Verdict::ALLOW: return "ALLOW";
    case Verdict::CLAMP: return "CLAMP";
    case Verdict::VETO: return "VETO";
  }
  return "UNKNOWN";
}

std::string toString(Command::Kind kind)
{
  switch (kind) {
    case Command::Kind::POSE: return "pose";
    case Command::Kind::TWIST: return "twist";
    case Command::Kind::TRAJECTORY: return "trajectory";
    case Command::Kind::SPEED_LIMIT: return "speed_limit";
  }
  return "unknown";
}

Governor::Governor(std::vector<PolicyPtr> policies, GovernorConfig config)
: policies_(std::move(policies)), config_(config)
{
}

void Governor::updateState(const State & state)
{
  state_ = state;
  has_state_ = true;
}

Decision Governor::evaluate(const Command & command, double now)
{
  if (latched_) {
    Decision decision;
    decision.verdict = Verdict::VETO;
    decision.latched = true;
    decision.requested = command;
    decision.allowed = holdCommand(command.kind);
    decision.allowed_valid = holdIsValid(command.kind);
    decision.policies = latched_policies_;
    decision.reason = "latched: " + latched_reason_;
    return decision;
  }

  if (!has_state_) {
    return makeVeto(command, {}, "no vehicle state received");
  }
  const double age = now - state_.stamp;
  if (!std::isfinite(age) || age > config_.state_timeout) {
    return makeVeto(
      command, {}, "vehicle state is stale (" + std::to_string(age) + " s old)");
  }
  if (age < -config_.state_timeout) {
    // A state from the future means the clocks disagree (e.g. use_sim_time is
    // set on some nodes only), so staleness cannot be judged.
    return makeVeto(
      command, {}, "vehicle state is stamped " + std::to_string(-age) +
      " s in the future: check use_sim_time");
  }

  const bool bad_limit = command.isSpeedLimit() &&
    (std::isnan(command.velocity.x) || std::isnan(command.velocity.y) ||
    std::isnan(command.velocity.z) || command.velocity.x < 0.0 ||
    command.velocity.y < 0.0 || command.velocity.z < 0.0);
  if ((command.hasPosition() && !isFinite(command.position)) ||
    (command.hasVelocity() && !isFinite(command.velocity)) || bad_limit)
  {
    return makeVeto(command, {}, "command contains non-finite values");
  }
  if (!isFinite(state_.position)) {
    return makeVeto(command, {}, "vehicle state contains non-finite values");
  }

  Decision decision;
  decision.requested = command;
  Command current = command;
  for (const auto & policy : policies_) {
    PolicyResult result = policy->apply(current, state_);
    if (result.verdict == Verdict::VETO) {
      std::vector<std::string> acting = decision.policies;
      acting.push_back(policy->name());
      return makeVeto(command, acting, policy->name() + ": " + result.reason);
    }
    if (result.verdict == Verdict::CLAMP) {
      decision.verdict = Verdict::CLAMP;
      decision.policies.push_back(policy->name());
      if (!decision.reason.empty()) {decision.reason += "; ";}
      decision.reason += policy->name() + ": " + result.reason;
      current = result.command;
    }
  }
  decision.allowed = current;
  return decision;
}

Decision Governor::veto(const Command & command, const std::string & reason)
{
  if (latched_) {
    return evaluate(command, state_.stamp);
  }
  return makeVeto(command, {}, reason);
}

bool Governor::reset()
{
  const bool was_latched = latched_;
  latched_ = false;
  latched_reason_.clear();
  latched_policies_.clear();
  return was_latched;
}

Decision Governor::makeVeto(
  const Command & command, std::vector<std::string> policies,
  const std::string & reason)
{
  // The hold point is fixed at the moment of the veto, so a latched hold does
  // not drift with later (possibly bad) state estimates.
  if (has_state_ && isFinite(state_.position)) {
    hold_position_ = state_.position;
    has_hold_position_ = true;
  }
  if (config_.latch_veto) {
    latched_ = true;
    latched_reason_ = reason;
    latched_policies_ = policies;
  }

  Decision decision;
  decision.verdict = Verdict::VETO;
  decision.latched = latched_;
  decision.requested = command;
  decision.allowed = holdCommand(command.kind);
  decision.allowed_valid = holdIsValid(command.kind);
  decision.policies = std::move(policies);
  decision.reason = reason;
  return decision;
}

Command Governor::holdCommand(Command::Kind kind) const
{
  Command hold;
  hold.kind = kind;
  hold.position = hold_position_;
  hold.velocity = kind == Command::Kind::SPEED_LIMIT ?
    Vec3{config_.hold_speed, config_.hold_speed, config_.hold_speed} : Vec3{};
  return hold;
}

bool Governor::holdIsValid(Command::Kind kind) const
{
  // A zero-velocity hold is always meaningful; a position hold needs a position.
  return kind == Command::Kind::TWIST || kind == Command::Kind::SPEED_LIMIT ||
         has_hold_position_;
}

}  // namespace as2_flight_governor
