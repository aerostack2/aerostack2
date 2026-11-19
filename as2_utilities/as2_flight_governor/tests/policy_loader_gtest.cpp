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
* @file policy_loader_gtest.cpp
*
* Tests of the YAML policy loader.
*
* @author Vishwa
*/

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "as2_flight_governor/policy_loader.hpp"

namespace afg = as2_flight_governor;

namespace
{

std::vector<afg::PolicyPtr> load(const std::string & text)
{
  return afg::loadPolicies(YAML::Load(text));
}

void expectError(const std::string & text, const std::string & fragment)
{
  try {
    load(text);
    FAIL() << "expected an error containing '" << fragment << "'";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find(fragment), std::string::npos) << e.what();
  }
}

}  // namespace

TEST(PolicyLoader, LoadsExampleFileInOrder) {
  const auto policies = afg::loadPoliciesFromFile(std::string(CONFIG_DIR) + "/policies.yaml");
  ASSERT_EQ(policies.size(), 4u);
  EXPECT_EQ(policies[0]->name(), "arena");
  EXPECT_EQ(policies[0]->type(), "geofence_box");
  EXPECT_EQ(policies[1]->type(), "max_speed");
  EXPECT_EQ(policies[2]->type(), "keepout_sphere");
  EXPECT_EQ(policies[3]->type(), "speed_zone");
}

TEST(PolicyLoader, AppliesDefaults) {
  const auto policies = load(
    "policies:\n"
    "  - {name: b, type: geofence_box, min: [0, 0, 0], max: [1, 1, 1]}\n");
  ASSERT_EQ(policies.size(), 1u);
}

TEST(PolicyLoader, EmptyListIsAllowed) {
  EXPECT_TRUE(load("policies: []\n").empty());
}

TEST(PolicyLoader, RejectsMissingList) {
  expectError("limits: []\n", "'policies' list");
}

TEST(PolicyLoader, RejectsUnknownType) {
  expectError("policies:\n  - {name: a, type: teleport}\n", "unknown type 'teleport'");
}

TEST(PolicyLoader, RejectsTyposInFields) {
  // A misspelt optional field must not silently fall back to its default.
  expectError(
    "policies:\n"
    "  - {name: b, type: geofence_box, min: [0, 0, 0], max: [1, 1, 1], lookahaed: 5}\n",
    "unknown field 'lookahaed'");
}

TEST(PolicyLoader, RejectsMissingField) {
  expectError(
    "policies:\n  - {name: k, type: keepout_sphere, center: [0, 0, 0]}\n",
    "missing required field 'radius'");
}

TEST(PolicyLoader, RejectsMalformedVector) {
  expectError(
    "policies:\n  - {name: b, type: geofence_box, min: [0, 0], max: [1, 1, 1]}\n",
    "list of 3 numbers");
}

TEST(PolicyLoader, RejectsNonNumber) {
  expectError(
    "policies:\n  - {name: s, type: max_speed, horizontal: fast}\n",
    "'horizontal' must be a number");
}

TEST(PolicyLoader, RejectsBadAction) {
  expectError(
    "policies:\n"
    "  - {name: b, type: geofence_box, min: [0, 0, 0], max: [1, 1, 1], action: ignore}\n",
    "'clamp' or 'veto'");
}

TEST(PolicyLoader, RejectsInvalidGeometry) {
  expectError(
    "policies:\n  - {name: b, type: geofence_box, min: [0, 0, 0], max: [1, -1, 1]}\n",
    "policy 'b'");
}

TEST(PolicyLoader, RejectsDuplicateNames) {
  expectError(
    "policies:\n"
    "  - {name: s, type: max_speed, horizontal: 1}\n"
    "  - {name: s, type: max_speed, vertical: 1}\n",
    "duplicate name");
}

TEST(PolicyLoader, RejectsUnreadableFile) {
  EXPECT_THROW(afg::loadPoliciesFromFile("/nonexistent/policies.yaml"), std::runtime_error);
}
