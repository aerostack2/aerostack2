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
* @file governor_core_gtest.cpp
*
* Tests of the ROS-independent flight governor core and policies.
*
* @author Vishwa
*/

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "as2_flight_governor/governor_core.hpp"
#include "as2_flight_governor/policies.hpp"

namespace afg = as2_flight_governor;
using afg::Command;
using afg::Decision;
using afg::Governor;
using afg::GovernorConfig;
using afg::State;
using afg::Vec3;
using afg::Verdict;

namespace
{

constexpr double kTol = 1e-9;
constexpr double kInf = std::numeric_limits<double>::infinity();

Command twist(double vx, double vy, double vz)
{
  Command c;
  c.kind = Command::Kind::TWIST;
  c.velocity = Vec3{vx, vy, vz};
  return c;
}

Command pose(double x, double y, double z)
{
  Command c;
  c.kind = Command::Kind::POSE;
  c.position = Vec3{x, y, z};
  return c;
}

Command speedLimit(double vx, double vy, double vz)
{
  Command c;
  c.kind = Command::Kind::SPEED_LIMIT;
  c.velocity = Vec3{vx, vy, vz};
  return c;
}

State at(double x, double y, double z, double stamp = 10.0)
{
  return State{Vec3{x, y, z}, stamp};
}

std::shared_ptr<afg::GeofenceBoxPolicy> box(afg::Action action = afg::Action::CLAMP)
{
  return std::make_shared<afg::GeofenceBoxPolicy>(
    "box", Vec3{-10, -10, 0}, Vec3{10, 10, 5}, 1.0, action);
}

Governor governorWith(std::vector<afg::PolicyPtr> policies, bool latch = true)
{
  GovernorConfig config;
  config.state_timeout = 0.5;
  config.latch_veto = latch;
  return Governor(std::move(policies), config);
}

void expectVec(const Vec3 & v, double x, double y, double z)
{
  EXPECT_NEAR(v.x, x, kTol);
  EXPECT_NEAR(v.y, y, kTol);
  EXPECT_NEAR(v.z, z, kTol);
}

}  // namespace

// ---------------------------------------------------------------------------
// Governor

TEST(Governor, AllowLeavesCommandUnchanged) {
  Governor g = governorWith({box()});
  g.updateState(at(0, 0, 1));
  const Decision d = g.evaluate(twist(1, 0, 0), 10.1);
  EXPECT_EQ(d.verdict, Verdict::ALLOW);
  EXPECT_TRUE(d.policies.empty());
  expectVec(d.allowed.velocity, 1, 0, 0);
}

TEST(Governor, VetoesWithoutState) {
  Governor g = governorWith({box()});
  const Decision d = g.evaluate(twist(1, 0, 0), 10.0);
  EXPECT_EQ(d.verdict, Verdict::VETO);
  EXPECT_TRUE(d.allowed_valid);  // Zero velocity is always a valid hold.
  expectVec(d.allowed.velocity, 0, 0, 0);
}

TEST(Governor, PositionHoldWithoutStateSendsNothing) {
  Governor g = governorWith({box()});
  const Decision d = g.evaluate(pose(1, 1, 1), 10.0);
  EXPECT_EQ(d.verdict, Verdict::VETO);
  EXPECT_FALSE(d.allowed_valid);  // Never invent a hold point at the origin.
}

TEST(Governor, VetoesStaleState) {
  Governor g = governorWith({box()});
  g.updateState(at(0, 0, 1, 10.0));
  EXPECT_EQ(g.evaluate(twist(1, 0, 0), 10.4).verdict, Verdict::ALLOW);
  Governor g2 = governorWith({box()});
  g2.updateState(at(0, 0, 1, 10.0));
  EXPECT_EQ(g2.evaluate(twist(1, 0, 0), 10.6).verdict, Verdict::VETO);
}

TEST(Governor, VetoesStateFromTheFuture) {
  // Mixed sim time and wall time: the state can look fresh forever.
  Governor g = governorWith({box()});
  g.updateState(at(0, 0, 1, 1000.0));
  EXPECT_EQ(g.evaluate(twist(1, 0, 0), 10.0).verdict, Verdict::VETO);
}

TEST(Governor, VetoesNonFiniteCommand) {
  Governor g = governorWith({});
  g.updateState(at(0, 0, 1));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(g.evaluate(twist(nan, 0, 0), 10.0).verdict, Verdict::VETO);
}

TEST(Governor, VetoesInvalidSpeedLimit) {
  Governor g = governorWith({}, false);
  g.updateState(at(0, 0, 1));
  EXPECT_EQ(g.evaluate(speedLimit(-1, 1, 1), 10.0).verdict, Verdict::VETO);
  EXPECT_EQ(
    g.evaluate(speedLimit(std::numeric_limits<double>::quiet_NaN(), 1, 1), 10.0).verdict,
    Verdict::VETO);
  EXPECT_EQ(g.evaluate(speedLimit(kInf, 1, 1), 10.0).verdict, Verdict::ALLOW);
}

TEST(Governor, VetoLatchesUntilReset) {
  Governor g = governorWith({box(afg::Action::VETO)});
  g.updateState(at(9.5, 0, 1));
  EXPECT_EQ(g.evaluate(twist(3, 0, 0), 10.0).verdict, Verdict::VETO);
  EXPECT_TRUE(g.isLatched());

  // A harmless command is still vetoed while latched.
  const Decision latched = g.evaluate(twist(-1, 0, 0), 10.1);
  EXPECT_EQ(latched.verdict, Verdict::VETO);
  EXPECT_TRUE(latched.latched);
  EXPECT_EQ(latched.policies, std::vector<std::string>{"box"});

  EXPECT_TRUE(g.reset());
  EXPECT_FALSE(g.isLatched());
  EXPECT_EQ(g.evaluate(twist(-1, 0, 0), 10.2).verdict, Verdict::ALLOW);
  EXPECT_FALSE(g.reset());
}

TEST(Governor, UnlatchedVetoReleasesOnNextGoodCommand) {
  Governor g = governorWith({box(afg::Action::VETO)}, false);
  g.updateState(at(9.5, 0, 1));
  EXPECT_EQ(g.evaluate(twist(3, 0, 0), 10.0).verdict, Verdict::VETO);
  EXPECT_FALSE(g.isLatched());
  EXPECT_EQ(g.evaluate(twist(-1, 0, 0), 10.1).verdict, Verdict::ALLOW);
}

TEST(Governor, HoldPointIsFixedAtVetoTime) {
  Governor g = governorWith({box(afg::Action::VETO)});
  g.updateState(at(9.5, 1, 1, 10.0));
  g.evaluate(pose(20, 0, 1), 10.0);
  g.updateState(at(3, 3, 3, 10.1));  // Later state must not move the hold.
  const Decision d = g.evaluate(pose(0, 0, 1), 10.1);
  EXPECT_EQ(d.verdict, Verdict::VETO);
  expectVec(d.allowed.position, 9.5, 1, 1);
}

TEST(Governor, SpeedLimitHoldUsesHoldSpeed) {
  GovernorConfig config;
  config.hold_speed = 0.3;
  Governor g({}, config);
  const Decision d = g.evaluate(speedLimit(2, 2, 2), 10.0);  // No state: veto.
  EXPECT_EQ(d.verdict, Verdict::VETO);
  EXPECT_TRUE(d.allowed_valid);
  expectVec(d.allowed.velocity, 0.3, 0.3, 0.3);
}

TEST(Governor, ClampsComposeInOrder) {
  auto speed = std::make_shared<afg::MaxSpeedPolicy>("speed", 1.0, -1.0);
  Governor g = governorWith({box(), speed});
  g.updateState(at(9.5, 0, 1));
  const Decision d = g.evaluate(twist(3, 3, 0), 10.0);
  EXPECT_EQ(d.verdict, Verdict::CLAMP);
  EXPECT_EQ(d.policies, (std::vector<std::string>{"box", "speed"}));
  // Box limits vx to 0.5, then the horizontal speed (0.5, 3) is scaled to 1.
  EXPECT_NEAR(std::hypot(d.allowed.velocity.x, d.allowed.velocity.y), 1.0, kTol);
  EXPECT_NEAR(d.allowed.velocity.y / d.allowed.velocity.x, 6.0, 1e-6);
  expectVec(d.requested.velocity, 3, 3, 0);
}

TEST(Governor, IsDeterministic) {
  for (int run = 0; run < 2; ++run) {
    Governor g = governorWith({box()});
    g.updateState(at(9.0, -9.8, 4.9));
    const Decision d = g.evaluate(twist(4, -2, 1), 10.2);
    EXPECT_EQ(d.verdict, Verdict::CLAMP);
    expectVec(d.allowed.velocity, 1.0, -0.2, 0.1);
  }
}

// ---------------------------------------------------------------------------
// GeofenceBoxPolicy

TEST(GeofenceBox, ClampsPositionIntoBox) {
  const auto result = box()->apply(pose(20, -15, 7), at(0, 0, 1));
  EXPECT_EQ(result.verdict, Verdict::CLAMP);
  expectVec(result.command.position, 10, -10, 5);
}

TEST(GeofenceBox, LimitsVelocityNearFace) {
  const auto result = box()->apply(twist(3, 0, 0), at(9, 0, 1));
  EXPECT_EQ(result.verdict, Verdict::CLAMP);
  expectVec(result.command.velocity, 1, 0, 0);
}

TEST(GeofenceBox, OutsideAllowsReturnOnly) {
  const auto out = box()->apply(twist(1, 0, 0), at(11, 0, 1));
  EXPECT_EQ(out.verdict, Verdict::CLAMP);
  expectVec(out.command.velocity, 0, 0, 0);
  const auto back = box()->apply(twist(-1, 0, 0), at(11, 0, 1));
  EXPECT_EQ(back.verdict, Verdict::ALLOW);
}

TEST(GeofenceBox, IgnoresSpeedLimits) {
  EXPECT_EQ(box()->apply(speedLimit(5, 5, 5), at(9.9, 0, 1)).verdict, Verdict::ALLOW);
}

TEST(GeofenceBox, RejectsBadConfiguration) {
  EXPECT_THROW(
    afg::GeofenceBoxPolicy("b", Vec3{0, 0, 0}, Vec3{0, 1, 1}), std::invalid_argument);
  EXPECT_THROW(
    afg::GeofenceBoxPolicy("b", Vec3{0, 0, 0}, Vec3{1, 1, 1}, 0.0), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// MaxSpeedPolicy

TEST(MaxSpeed, ScalesHorizontalKeepingDirection) {
  afg::MaxSpeedPolicy p("s", 1.0, 0.5);
  const auto result = p.apply(twist(3, 4, -2), at(0, 0, 1));
  EXPECT_EQ(result.verdict, Verdict::CLAMP);
  expectVec(result.command.velocity, 0.6, 0.8, -0.5);
}

TEST(MaxSpeed, AllowsSlowCommands) {
  afg::MaxSpeedPolicy p("s", 1.0, 0.5);
  EXPECT_EQ(p.apply(twist(0.5, 0.5, 0.1), at(0, 0, 1)).verdict, Verdict::ALLOW);
  EXPECT_EQ(p.apply(pose(100, 0, 0), at(0, 0, 1)).verdict, Verdict::ALLOW);
}

TEST(MaxSpeed, CapsUnlimitedSpeedLimits) {
  afg::MaxSpeedPolicy p("s", 1.0, 0.5);
  const auto result = p.apply(speedLimit(kInf, kInf, kInf), at(0, 0, 1));
  EXPECT_EQ(result.verdict, Verdict::CLAMP);
  // Each horizontal axis may be saturated at once, so the pair is scaled.
  EXPECT_NEAR(std::hypot(result.command.velocity.x, result.command.velocity.y), 1.0, kTol);
  EXPECT_NEAR(result.command.velocity.z, 0.5, kTol);
}

TEST(MaxSpeed, VerticalOnlyLeavesHorizontalUnlimited) {
  afg::MaxSpeedPolicy p("s", -1.0, 0.5);
  const auto result = p.apply(speedLimit(kInf, kInf, kInf), at(0, 0, 1));
  EXPECT_TRUE(std::isinf(result.command.velocity.x));
  EXPECT_NEAR(result.command.velocity.z, 0.5, kTol);
}

TEST(MaxSpeed, RejectsAllDisabled) {
  EXPECT_THROW(afg::MaxSpeedPolicy("s", -1.0, -1.0), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// KeepoutSpherePolicy

TEST(KeepoutSphere, ProjectsTargetOntoSurface) {
  afg::KeepoutSpherePolicy p("k", Vec3{0, 0, 2}, 1.0, 0.5);
  const auto result = p.apply(pose(0.5, 0, 2), at(-5, 0, 2));
  EXPECT_EQ(result.verdict, Verdict::CLAMP);
  expectVec(result.command.position, 1.5, 0, 2);
}

TEST(KeepoutSphere, TargetAtCentreUsesVehicleSide) {
  afg::KeepoutSpherePolicy p("k", Vec3{0, 0, 2}, 1.0);
  const auto result = p.apply(pose(0, 0, 2), at(-5, 0, 2));
  expectVec(result.command.position, -1, 0, 2);
}

TEST(KeepoutSphere, LimitsInwardKeepsTangential) {
  afg::KeepoutSpherePolicy p("k", Vec3{0, 0, 2}, 1.0, 0.0, 1.0);
  // 2 m from the centre, 1 m from the surface: at most 1 m/s inwards.
  const auto result = p.apply(twist(3, 2, 0), at(-2, 0, 2));
  EXPECT_EQ(result.verdict, Verdict::CLAMP);
  expectVec(result.command.velocity, 1, 2, 0);
}

TEST(KeepoutSphere, AllowsMovingAway) {
  afg::KeepoutSpherePolicy p("k", Vec3{0, 0, 2}, 1.0);
  EXPECT_EQ(p.apply(twist(-3, 0, 0), at(-1.5, 0, 2)).verdict, Verdict::ALLOW);
}

TEST(KeepoutSphere, VetoAction) {
  afg::KeepoutSpherePolicy p("k", Vec3{0, 0, 2}, 1.0, 0.0, 1.0, afg::Action::VETO);
  EXPECT_EQ(p.apply(twist(3, 0, 0), at(-2, 0, 2)).verdict, Verdict::VETO);
}

// ---------------------------------------------------------------------------
// SpeedZonePolicy

TEST(SpeedZone, LimitsOnlyInsideZone) {
  afg::SpeedZonePolicy p("z", Vec3{0, 0, 0}, 2.0, 0.5);
  const auto inside = p.apply(twist(3, 0, 4), at(1, 0, 0));
  EXPECT_EQ(inside.verdict, Verdict::CLAMP);
  expectVec(inside.command.velocity, 0.3, 0, 0.4);
  EXPECT_EQ(p.apply(twist(3, 0, 4), at(5, 0, 0)).verdict, Verdict::ALLOW);
}

TEST(SpeedZone, CapsUnlimitedSpeedLimits) {
  afg::SpeedZonePolicy p("z", Vec3{0, 0, 0}, 2.0, 0.5);
  const auto result = p.apply(speedLimit(kInf, 0.1, kInf), at(0, 0, 0));
  EXPECT_EQ(result.verdict, Verdict::CLAMP);
  EXPECT_NEAR(afg::norm(result.command.velocity), 0.5, kTol);
  EXPECT_FALSE(std::isinf(result.command.velocity.x));
}
