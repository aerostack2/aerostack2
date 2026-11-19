// Copyright 2026 Universidad Politécnica de Madrid
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
* @file governor_core.hpp
*
* ROS-independent core of the flight governor: commands, state, policies and
* the governor that composes them into an ALLOW / CLAMP / VETO decision.
*
* The core never reads a clock. The caller passes the current time in, so the
* same inputs always produce the same decision.
*
* @author Vishwa Gunawardana
*/

#ifndef AS2_FLIGHT_GOVERNOR__GOVERNOR_CORE_HPP_
#define AS2_FLIGHT_GOVERNOR__GOVERNOR_CORE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace as2_flight_governor
{

/// Minimal 3D vector, kept local so the core has no dependencies.
struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vec3 operator+(const Vec3 & a, const Vec3 & b);
Vec3 operator-(const Vec3 & a, const Vec3 & b);
Vec3 operator*(const Vec3 & a, double s);
double dot(const Vec3 & a, const Vec3 & b);
double norm(const Vec3 & a);

/// Outcome of a policy or of the whole governor. Ordered by severity.
enum class Verdict : std::uint8_t
{
  ALLOW = 0,
  CLAMP = 1,
  VETO = 2,
};

std::string toString(Verdict verdict);

/// One actuator command, expressed in the policy frame.
struct Command
{
  enum class Kind : std::uint8_t
  {
    POSE,        ///< Position setpoint (orientation is never modified).
    TWIST,       ///< Linear velocity setpoint (angular part is never modified).
    TRAJECTORY,  ///< Trajectory setpoint: position and velocity feed-forward.
    /// Per-axis speed limits (absolute values) that accompany a position
    /// reference. Stored in `velocity`; +infinity means "no limit" on that axis.
    SPEED_LIMIT,
  };

  Kind kind = Kind::TWIST;
  Vec3 position;  ///< Used by POSE and TRAJECTORY.
  Vec3 velocity;  ///< Used by TWIST and TRAJECTORY.

  bool hasPosition() const {return kind == Kind::POSE || kind == Kind::TRAJECTORY;}
  bool hasVelocity() const {return kind == Kind::TWIST || kind == Kind::TRAJECTORY;}
  bool isSpeedLimit() const {return kind == Kind::SPEED_LIMIT;}
};

std::string toString(Command::Kind kind);

/// Vehicle state, expressed in the policy frame.
struct State
{
  Vec3 position;
  double stamp = 0.0;  ///< Seconds, same clock as the `now` given to the governor.
};

/// What a single policy decided about a command.
struct PolicyResult
{
  Verdict verdict = Verdict::ALLOW;
  Command command;   ///< The (possibly clamped) command.
  std::string reason;
};

/// A declarative safety rule. Policies are pure: same inputs, same result.
class Policy
{
public:
  explicit Policy(std::string name)
  : name_(std::move(name)) {}
  virtual ~Policy() = default;

  const std::string & name() const {return name_;}
  virtual std::string type() const = 0;

  /// Evaluate the command against the current state.
  virtual PolicyResult apply(const Command & command, const State & state) const = 0;

private:
  std::string name_;
};

using PolicyPtr = std::shared_ptr<const Policy>;

/// The governor's decision for one command.
struct Decision
{
  Verdict verdict = Verdict::ALLOW;
  bool latched = false;               ///< True while a latched veto is active.
  Command requested;                  ///< What the controller asked for.
  Command allowed;                    ///< What must be sent to the platform.
  /// False when no safe command exists (a position hold requested before any
  /// state was received). The caller must then send nothing.
  bool allowed_valid = true;
  std::vector<std::string> policies;  ///< Policies that clamped or vetoed.
  std::string reason;
};

struct GovernorConfig
{
  /// Maximum age of the state, in seconds. Older (or missing) state is a veto.
  double state_timeout = 0.5;
  /// If true, a veto holds until reset() is called.
  bool latch_veto = true;
  /// Per-axis speed limit, in m/s, sent with a position hold after a veto.
  double hold_speed = 0.5;
};

/**
 * Composes policies in order. Each policy sees the output of the previous one,
 * so clamps compose. The first veto stops evaluation. On veto the allowed
 * command is a hold: zero velocity, the position at the moment of the veto,
 * and `hold_speed` as the speed limit of a position hold.
 */
class Governor
{
public:
  Governor(std::vector<PolicyPtr> policies, GovernorConfig config);

  /// Govern one command. `now` is in seconds, on the same clock as State::stamp.
  Decision evaluate(const Command & command, double now);

  /// Update the vehicle state used by subsequent evaluations.
  void updateState(const State & state);

  /// Veto for a reason outside the policies (e.g. a failed frame transform).
  Decision veto(const Command & command, const std::string & reason);

  /// Clear a latched veto. Returns true if a veto was latched.
  bool reset();

  bool isLatched() const {return latched_;}
  const std::vector<PolicyPtr> & policies() const {return policies_;}
  const GovernorConfig & config() const {return config_;}

private:
  Decision makeVeto(
    const Command & command, std::vector<std::string> policies,
    const std::string & reason);
  Command holdCommand(Command::Kind kind) const;
  bool holdIsValid(Command::Kind kind) const;

  std::vector<PolicyPtr> policies_;
  GovernorConfig config_;

  bool has_state_ = false;
  State state_;

  bool latched_ = false;
  std::string latched_reason_;
  std::vector<std::string> latched_policies_;
  bool has_hold_position_ = false;
  Vec3 hold_position_;
};

}  // namespace as2_flight_governor

#endif  // AS2_FLIGHT_GOVERNOR__GOVERNOR_CORE_HPP_
