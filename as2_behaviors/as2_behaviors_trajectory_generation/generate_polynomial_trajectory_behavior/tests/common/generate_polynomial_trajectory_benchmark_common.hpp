// Copyright 2026 Universidad Politecnica de Madrid
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
//    * Neither the name of the Universidad Politecnica de Madrid nor the names
//    of its
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
 * @file generate_polynomial_trajectory_benchmark_common.hpp
 *
 * @brief Shared helpers for Google-Benchmark suites that compare polynomial
 * trajectory generator plugins on the public plugin API
 * (generateTrajectory / updateWaypoints / evaluate), parametrized over the
 * number of mission waypoints.
 *
 * Header-only so both the comparative root binary and the per-plugin
 * standalone binaries can include it without an extra link target.
 *
 * @authors Rafael Perez-Segui
 */

#ifndef GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__BENCHMARK_COMMON_HPP_  // NOLINT
#define GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__BENCHMARK_COMMON_HPP_  // NOLINT

#include <benchmark/benchmark.h>
#include <rcutils/logging.h>

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/msg/trajectory_point.hpp"

#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_base.hpp"

namespace generate_polynomial_trajectory_benchmark
{

using PluginBase = generate_polynomial_trajectory_behavior_plugin_base::
  GeneratePolynomialTrajectoryBase;

// Synthetic mission constants used by all benchmarks. Centralized so a
// reader does not need to chase magic numbers across helpers.
constexpr double kBenchmarkRadius = 2.0;       // [m] helix radius
constexpr double kBenchmarkPitchZ = 0.5;       // [m] vertical step per waypoint
constexpr double kBenchmarkBaseZ = 1.0;        // [m] starting altitude
constexpr double kBenchmarkMaxSpeed = 1.0;     // [m/s] cruise speed

// Per-benchmark execution budget. Google Benchmark adapts the iteration
// count so each registered benchmark runs for at least this long before
// reporting; sub-microsecond calls still get a stable estimate, while
// multi-millisecond calls cap at one iteration. Override on the command
// line with --benchmark_min_time=<seconds>s.
constexpr double kBenchmarkMinTimeSeconds = 0.1;

/**
 * @brief Raise the default rclcpp log severity threshold for the running
 * process so per-iteration INFO logs (e.g. the parameter dumps emitted by
 * the plugin base on every getParameter call) do not flood the benchmark
 * output. WARN keeps genuine warnings/errors visible.
 *
 * Note: this only affects rclcpp/rcutils-based logging. Backends that
 * write directly to stdout/stderr (e.g. dynamic_trajectory_generator) are
 * not routed through rosout — for those, see ScopedStdoutSink below.
 *
 * @param severity One of RCUTILS_LOG_SEVERITY_*.
 */
inline void silenceRosLogging(int severity = RCUTILS_LOG_SEVERITY_WARN)
{
  rcutils_logging_set_default_logger_level(severity);
}

/**
 * @brief RAII guard that redirects file descriptor 1 (stdout) to /dev/null
 * for its lifetime, restoring the original target on destruction.
 *
 * Used inside each benchmark lambda to suppress non-rosout chatter from
 * vendored backends that bypass rclcpp and write directly to stdout
 * (printf / std::cout). Google Benchmark prints the final result row
 * AFTER the lambda returns, so the table itself stays visible while the
 * per-iteration backend chatter is dropped.
 *
 * Linux/POSIX-only — relies on dup/dup2/open. Not thread-safe; benchmarks
 * here are single-threaded so this is fine.
 */
class ScopedStdoutSink
{
public:
  ScopedStdoutSink()
  {
    std::fflush(stdout);
    saved_fd_ = ::dup(STDOUT_FILENO);
    null_fd_ = ::open("/dev/null", O_WRONLY);
    if (null_fd_ >= 0) {
      ::dup2(null_fd_, STDOUT_FILENO);
    }
  }

  ~ScopedStdoutSink()
  {
    std::fflush(stdout);
    if (saved_fd_ >= 0) {
      ::dup2(saved_fd_, STDOUT_FILENO);
      ::close(saved_fd_);
    }
    if (null_fd_ >= 0) {
      ::close(null_fd_);
    }
  }

  ScopedStdoutSink(const ScopedStdoutSink &) = delete;
  ScopedStdoutSink & operator=(const ScopedStdoutSink &) = delete;

private:
  int saved_fd_{-1};
  int null_fd_{-1};
};

inline as2_msgs::msg::PoseStampedWithID makeWaypoint(
  const std::string & id, double x, double y, double z)
{
  as2_msgs::msg::PoseStampedWithID wp;
  wp.id = id;
  wp.pose.header.frame_id = "earth";
  wp.pose.pose.position.x = x;
  wp.pose.pose.position.y = y;
  wp.pose.pose.position.z = z;
  wp.pose.pose.orientation.w = 1.0;
  return wp;
}

inline geometry_msgs::msg::PoseStamped makePose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "earth";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation.w = 1.0;
  return p;
}

/**
 * @brief Build N synthetic waypoints distributed along a 3D helix.
 *
 * The helix has constant radius and constant vertical pitch per waypoint,
 * with one full revolution every N waypoints. Increasing N therefore keeps
 * total path length bounded while adding intermediate corners — which is
 * exactly the dimension we want to scale when benchmarking how plugins
 * handle larger waypoint sets.
 *
 * @param n Number of waypoints (clamped to >= 1 internally for the angular
 *          division; an empty vector is returned when n == 0).
 * @param radius Helix radius in metres.
 * @param pitch_z Vertical step between consecutive waypoints in metres.
 * @param x_offset Optional translation along x; used by updateWaypoints
 *                 benchmarks to feed a slightly displaced replacement set.
 */
inline std::vector<as2_msgs::msg::PoseStampedWithID>
generateHelixWaypoints(
  std::size_t n,
  double radius = kBenchmarkRadius,
  double pitch_z = kBenchmarkPitchZ,
  double x_offset = 0.0)
{
  std::vector<as2_msgs::msg::PoseStampedWithID> wps;
  wps.reserve(n);
  const double two_pi = 2.0 * M_PI;
  const std::size_t denom = std::max<std::size_t>(n, 1U);
  for (std::size_t i = 0; i < n; ++i) {
    const double theta = two_pi * static_cast<double>(i + 1) /
      static_cast<double>(denom);
    const double x = x_offset + radius * std::cos(theta);
    const double y = radius * std::sin(theta);
    const double z = kBenchmarkBaseZ +
      static_cast<double>(i + 1) * pitch_z;
    char id[32];
    std::snprintf(id, sizeof(id), "waypoint_%03zu", i);
    wps.push_back(makeWaypoint(id, x, y, z));
  }
  return wps;
}

struct PluginFixture
{
  std::shared_ptr<as2::Node> node;
  std::shared_ptr<pluginlib::ClassLoader<PluginBase>> loader;
  std::shared_ptr<PluginBase> plugin;
};

/**
 * @brief Instantiate an as2::Node, load a plugin via pluginlib, and
 * initialize it. Vehicle state is set to the canonical origin pose with
 * zero twist so generateTrajectory has a deterministic start point.
 */
inline PluginFixture makePluginFixture(
  const std::string & plugin_name,
  const std::string & node_name,
  const std::vector<rclcpp::Parameter> & parameter_overrides = {})
{
  PluginFixture f;
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameter_overrides);
  f.node = std::make_shared<as2::Node>(node_name, options);
  f.loader = std::make_shared<pluginlib::ClassLoader<PluginBase>>(
    "as2_behaviors_trajectory_generation",
    "generate_polynomial_trajectory_behavior_plugin_base::"
    "GeneratePolynomialTrajectoryBase");
  f.plugin = f.loader->createSharedInstance(plugin_name + "::Plugin");
  f.plugin->initialize(f.node.get(), plugin_name);

  geometry_msgs::msg::TwistStamped zero_twist;
  zero_twist.header.frame_id = "earth";
  f.plugin->setVehicleState(makePose(0.0, 0.0, kBenchmarkBaseZ), zero_twist);
  return f;
}

/// Returns plugin names declared by pluginlib (strips the trailing "::Plugin").
inline std::vector<std::string> getAvailablePlugins()
{
  std::vector<std::string> names;
  try {
    pluginlib::ClassLoader<PluginBase> loader(
      "as2_behaviors_trajectory_generation",
      "generate_polynomial_trajectory_behavior_plugin_base::"
      "GeneratePolynomialTrajectoryBase");
    for (const auto & declared : loader.getDeclaredClasses()) {
      const std::string suffix = "::Plugin";
      if (declared.size() > suffix.size() &&
        declared.compare(
          declared.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        names.push_back(declared.substr(0, declared.size() - suffix.size()));
      } else {
        names.push_back(declared);
      }
    }
  } catch (const std::exception & ex) {
    std::fprintf(
      stderr, "getAvailablePlugins: pluginlib query failed: %s\n", ex.what());
  }
  return names;
}

inline std::string sanitizeForRosName(const std::string & s)
{
  std::string out = s;
  for (char & c : out) {
    if (!std::isalnum(static_cast<unsigned char>(c))) {c = '_';}
  }
  return out;
}

/**
 * @brief Register the three plugin-API benchmarks for a single plugin,
 * parametrized over the default sweep of waypoint counts.
 *
 * Registered names follow the convention:
 *   - BM_GenerateTrajectory/<plugin>/<N>
 *   - BM_UpdateWaypoints/<plugin>/<N>
 *   - BM_Evaluate/<plugin>/<N>
 *
 * Each benchmark also reports a `waypoints=N` counter so the user can
 * sort/group the resulting table by problem size.
 */
inline void registerPluginBenchmarks(const std::string & plugin_name)
{
  static const std::vector<int64_t> kWaypointCounts = {
    2, 5, 10, 20, 50, 100};
  const std::string sanitized = sanitizeForRosName(plugin_name);

  // ---- BM_GenerateTrajectory -------------------------------------------
  // One fixture per benchmark instance; each iteration resets the held
  // trajectory (excluded from timing) and times a fresh generateTrajectory
  // call. This mirrors the cold-start cost the host pays on activation.
  auto * gen_bm = benchmark::RegisterBenchmark(
    ("BM_GenerateTrajectory/" + plugin_name).c_str(),
    [plugin_name, sanitized](benchmark::State & state) {
      ScopedStdoutSink quiet_stdout;
      const auto n = static_cast<std::size_t>(state.range(0));
      const auto waypoints = generateHelixWaypoints(n);
      auto fx = makePluginFixture(
        plugin_name, "bm_gen_" + sanitized + "_node");
      for (auto _ : state) {
        state.PauseTiming();
        fx.plugin->reset();
        state.ResumeTiming();
        const bool ok = fx.plugin->generateTrajectory(
          waypoints, kBenchmarkMaxSpeed, /*t_trajectory_now=*/ 0.0);
        if (!ok) {
          state.SkipWithError("generateTrajectory returned false");
          break;
        }
        benchmark::DoNotOptimize(fx.plugin.get());
      }
      state.counters["waypoints"] = static_cast<double>(n);
    });
  for (int64_t n : kWaypointCounts) {
    gen_bm->Arg(n);
  }
  gen_bm->Unit(benchmark::kMicrosecond);
  gen_bm->MinTime(kBenchmarkMinTimeSeconds);

  // ---- BM_UpdateWaypoints ----------------------------------------------
  // Plugins that override updateWaypoints with online re-stitching keep
  // their internal time origin; plugins that rely on the base default
  // fall back to reset+regenerate. Both costs are surfaced apples to
  // apples in the resulting table — that comparison is the actual point
  // of this benchmark.
  auto * upd_bm = benchmark::RegisterBenchmark(
    ("BM_UpdateWaypoints/" + plugin_name).c_str(),
    [plugin_name, sanitized](benchmark::State & state) {
      ScopedStdoutSink quiet_stdout;
      const auto n = static_cast<std::size_t>(state.range(0));
      auto fx = makePluginFixture(
        plugin_name, "bm_upd_" + sanitized + "_node");
      const auto base_waypoints = generateHelixWaypoints(n);
      const bool ok0 = fx.plugin->generateTrajectory(
        base_waypoints, kBenchmarkMaxSpeed, /*t_trajectory_now=*/ 0.0);
      if (!ok0) {
        state.SkipWithError("initial generateTrajectory returned false");
        return;
      }
      const auto next_waypoints =
      generateHelixWaypoints(n, kBenchmarkRadius, kBenchmarkPitchZ, 0.1);
      for (auto _ : state) {
        const bool ok = fx.plugin->updateWaypoints(
          next_waypoints, kBenchmarkMaxSpeed, /*t_trajectory_now=*/ 0.0);
        if (!ok) {
          state.SkipWithError("updateWaypoints returned false");
          break;
        }
        benchmark::DoNotOptimize(fx.plugin.get());
      }
      state.counters["waypoints"] = static_cast<double>(n);
    });
  for (int64_t n : kWaypointCounts) {
    upd_bm->Arg(n);
  }
  upd_bm->Unit(benchmark::kMicrosecond);
  upd_bm->MinTime(kBenchmarkMinTimeSeconds);

  // ---- BM_Evaluate -----------------------------------------------------
  // Steady-state per-control-tick cost: a single evaluate at the
  // trajectory midpoint. Google Benchmark sets the iteration count
  // automatically to reach a stable measurement.
  auto * eval_bm = benchmark::RegisterBenchmark(
    ("BM_Evaluate/" + plugin_name).c_str(),
    [plugin_name, sanitized](benchmark::State & state) {
      ScopedStdoutSink quiet_stdout;
      const auto n = static_cast<std::size_t>(state.range(0));
      auto fx = makePluginFixture(
        plugin_name, "bm_eval_" + sanitized + "_node");
      const auto waypoints = generateHelixWaypoints(n);
      const bool ok0 = fx.plugin->generateTrajectory(
        waypoints, kBenchmarkMaxSpeed, /*t_trajectory_now=*/ 0.0);
      if (!ok0) {
        state.SkipWithError("generateTrajectory returned false");
        return;
      }
      const double duration = fx.plugin->getDuration();
      const double t_mid = 0.5 * duration;
      as2_msgs::msg::TrajectoryPoint out;
      for (auto _ : state) {
        const bool ok = fx.plugin->evaluate(
          t_mid, out, /*is_horizon_sample=*/ false);
        if (!ok) {
          state.SkipWithError("evaluate returned false");
          break;
        }
        benchmark::DoNotOptimize(out);
      }
      state.counters["waypoints"] = static_cast<double>(n);
    });
  for (int64_t n : kWaypointCounts) {
    eval_bm->Arg(n);
  }
  eval_bm->Unit(benchmark::kNanosecond);
  eval_bm->MinTime(kBenchmarkMinTimeSeconds);
}

}  // namespace generate_polynomial_trajectory_benchmark

#endif  // GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__BENCHMARK_COMMON_HPP_  // NOLINT
