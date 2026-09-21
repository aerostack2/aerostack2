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
* @file bindings.cpp
*
* nanobind module exposing simple_ekf_core::Filter, and the types it is driven with, to Python
*
* @authors Rodrigo Da Silva Gómez
*/

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <simple_ekf_core/filter.hpp>
#include <simple_ekf_core/measurement_utils.hpp>
#include <simple_ekf_core/rigid.hpp>
#include <simple_ekf_core/types.hpp>

namespace nb = nanobind;
using nanobind::literals::operator""_a;

using simple_ekf_core::Config;
using simple_ekf_core::Filter;
using simple_ekf_core::ImuSample;
using simple_ekf_core::LogLevel;
using simple_ekf_core::Nanoseconds;
using simple_ekf_core::Outputs;
using simple_ekf_core::PoseSample;
using simple_ekf_core::Quaternion;
using simple_ekf_core::Rigid;
using simple_ekf_core::SourceConfig;
using simple_ekf_core::SourceFrame;
using simple_ekf_core::SourceId;
using simple_ekf_core::TwistInBase;
using simple_ekf_core::TwistSample;
using simple_ekf_core::Vector3;

namespace
{

using Array3 = std::array<double, 3>;
using Array4 = std::array<double, 4>;
using Covariance6 = std::array<double, 36>;

Array3 toArray(const Vector3 & vector)
{
  return {vector.x(), vector.y(), vector.z()};
}

Vector3 toVector(const Array3 & array)
{
  return Vector3(array[0], array[1], array[2]);
}

/**
 * @brief Copy values into a numpy array that owns its buffer.
 *
 * A property returning one needs nb::rv_policy::automatic: the default for properties,
 * reference_internal, cannot be applied to an array that already has an owner.
 */
nb::ndarray<nb::numpy, double> toNumpy(std::vector<double> values, std::vector<size_t> shape)
{
  auto * buffer = new std::vector<double>(std::move(values));
  nb::capsule owner(
    buffer, [] (void * p) noexcept {delete static_cast<std::vector<double> *>(p);});
  return nb::ndarray<nb::numpy, double>(buffer->data(), shape.size(), shape.data(), owner);
}

std::string toString(const Array3 & array)
{
  return "(" + std::to_string(array[0]) + ", " + std::to_string(array[1]) + ", " +
         std::to_string(array[2]) + ")";
}

/**
 * @brief The filter's log sink: hands every line to the Python logger "simple_ekf_core".
 *
 * It looks the logger up on every line instead of keeping any Python object. A filter that
 * kept one referring back to the filter, as a callback or a log handler defined in the same
 * script does through the script's globals, would make a reference cycle the garbage collector
 * cannot see through C++, and it would never be freed.
 */
void logToPython(LogLevel level, const std::string & line)
{
  const char * level_names[] = {"INFO", "WARNING", "ERROR"};  // Indexed by LogLevel
  nb::module_ logging = nb::module_::import_("logging");
  logging.attr("getLogger")("simple_ekf_core").attr("log")(
    logging.attr(level_names[static_cast<std::size_t>(level)]), line);
}

}  // namespace

NB_MODULE(_simple_ekf_core, m) {
  m.doc() = "Python bindings for simple_ekf_core, the simple_ekf state estimator without ROS";

  nb::enum_<SourceFrame>(m, "SourceFrame", "The frame a measurement is expressed in.")
  .value("EARTH", SourceFrame::EARTH)
  .value("MAP", SourceFrame::MAP)
  .value("ODOM", SourceFrame::ODOM)
  .value("BASE", SourceFrame::BASE);

  nb::class_<Rigid>(
    m, "Transform",
    "A rigid transform: a position, and an orientation as a quaternion (x, y, z, w), the order "
    "ROS messages use.")
  .def(
    "__init__",
    [](Rigid * self, const Array3 & position, const Array4 & orientation) {
      new (self) Rigid(
        Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]),
        toVector(position));
    },
    "position"_a = Array3{0.0, 0.0, 0.0}, "orientation"_a = Array4{0.0, 0.0, 0.0, 1.0})
  .def_static(
    "from_rpy",
    [](const Array3 & position, double roll, double pitch, double yaw) {
      Quaternion rotation;
      rotation.setRPY(roll, pitch, yaw);
      return Rigid(rotation, toVector(position));
    },
    "position"_a, "roll"_a, "pitch"_a, "yaw"_a,
    "A transform from a position and roll, pitch, yaw in radians.")
  .def_prop_ro(
    "position", [](const Rigid & transform) {return toArray(transform.getOrigin());})
  .def_prop_ro(
    "orientation",
    [](const Rigid & transform) {
      const Quaternion rotation = transform.getRotation();
      return Array4{rotation.x(), rotation.y(), rotation.z(), rotation.w()};
    },
    "The orientation as a quaternion (x, y, z, w).")
  .def_prop_ro(
    "rpy",
    [](const Rigid & transform) {
      return simple_ekf_core::toRollPitchYaw(transform.getRotation());
    },
    "Roll, pitch and yaw, in radians, each in [-pi, pi].")
  .def(
    "matrix",
    [](const Rigid & transform) {
      std::vector<double> values(16, 0.0);
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          values[row * 4 + column] = transform.getBasis()[row][column];
        }
        values[row * 4 + 3] = transform.getOrigin()[row];
      }
      values[15] = 1.0;
      return toNumpy(std::move(values), {4, 4});
    },
    "The 4x4 homogeneous matrix.")
  .def("inverse", &Rigid::inverse)
  .def(
    "__mul__", [](const Rigid & first, const Rigid & second) {return first * second;},
    nb::is_operator())
  .def(
    "__repr__", [](const Rigid & transform) {
      return "Transform(position=" + toString(toArray(transform.getOrigin())) +
      ", rpy=" + toString(simple_ekf_core::toRollPitchYaw(transform.getRotation())) +
      ")";
    });

  nb::class_<SourceConfig>(m, "SourceConfig", "How the filter treats one source's measurements.")
  .def(nb::init<>())
  .def_rw("name", &SourceConfig::name)
  .def_rw("is_odometry", &SourceConfig::is_odometry)
  .def_rw("update_rate_hz", &SourceConfig::update_rate_hz)
  .def_rw("reject_repeated_positions", &SourceConfig::reject_repeated_positions)
  .def_rw("repeated_position_threshold", &SourceConfig::repeated_position_threshold)
  .def_rw("innovation_gate", &SourceConfig::innovation_gate)
  .def_rw("innovation_gate_timeout", &SourceConfig::innovation_gate_timeout)
  .def_rw("use_message_covariance", &SourceConfig::use_message_covariance)
  .def_rw("position_values", &SourceConfig::position_values)
  .def_rw("orientation_values", &SourceConfig::orientation_values)
  .def_rw("linear_values", &SourceConfig::linear_values)
  .def(
    "__repr__", [](const SourceConfig & config) {
      return "SourceConfig(name='" + config.name + "')";
    });

  nb::class_<Config>(m, "Config", "Everything the filter is set up with.")
  .def(nb::init<>())
  .def_rw("initial_position_covariance", &Config::initial_position_covariance)
  .def_rw("initial_velocity_covariance", &Config::initial_velocity_covariance)
  .def_rw("initial_orientation_covariance", &Config::initial_orientation_covariance)
  .def_rw("initial_bias_acc_covariance", &Config::initial_bias_acc_covariance)
  .def_rw("initial_bias_gyro_covariance", &Config::initial_bias_gyro_covariance)
  .def_rw("gravity", &Config::gravity)
  .def_rw("accelerometer_noise_density", &Config::accelerometer_noise_density)
  .def_rw("gyroscope_noise_density", &Config::gyroscope_noise_density)
  .def_rw("accelerometer_random_walk", &Config::accelerometer_random_walk)
  .def_rw("gyroscope_random_walk", &Config::gyroscope_random_walk)
  .def_rw("max_update_latency_ms", &Config::max_update_latency_ms)
  .def_rw("unobserved_variance", &Config::unobserved_variance)
  .def_rw("map_odom_alpha", &Config::map_odom_alpha)
  .def_rw("preflight_pose", &Config::preflight_pose)
  .def_rw("preflight_variance", &Config::preflight_variance)
  .def_rw("verbose", &Config::verbose)
  .def_rw("debug_verbose", &Config::debug_verbose);

  nb::class_<ImuSample>(
    m, "ImuSample", "One IMU reading, in the vehicle's frame. `stamp` is in nanoseconds.")
  .def(
    "__init__",
    [](ImuSample * self, Nanoseconds stamp, const Array3 & linear_acceleration,
    const Array3 & angular_velocity) {
      new (self) ImuSample{stamp, linear_acceleration, angular_velocity};
    },
    "stamp"_a = 0, "linear_acceleration"_a = Array3{0.0, 0.0, 0.0},
    "angular_velocity"_a = Array3{0.0, 0.0, 0.0})
  .def_rw("stamp", &ImuSample::stamp)
  .def_rw("linear_acceleration", &ImuSample::linear_acceleration)
  .def_rw("angular_velocity", &ImuSample::angular_velocity);

  nb::class_<PoseSample>(
    m, "PoseSample",
    "One pose measurement. `stamp` is in nanoseconds, `covariance` is 6x6 row-major, and a "
    "non-positive variance marks a component the source does not measure.")
  .def(
    "__init__",
    [](PoseSample * self, Nanoseconds stamp, SourceFrame frame, const Rigid & pose,
    const Covariance6 & covariance) {
      new (self) PoseSample{stamp, frame, pose, covariance};
    },
    "stamp"_a = 0, "frame"_a = SourceFrame::MAP, "pose"_a = Rigid::getIdentity(),
    "covariance"_a = Covariance6{})
  .def_rw("stamp", &PoseSample::stamp)
  .def_rw("frame", &PoseSample::frame)
  .def_rw("pose", &PoseSample::pose)
  .def_rw("covariance", &PoseSample::covariance);

  nb::class_<TwistSample>(
    m, "TwistSample",
    "One linear velocity measurement, with the same stamp and covariance conventions as "
    "PoseSample.")
  .def(
    "__init__",
    [](TwistSample * self, Nanoseconds stamp, SourceFrame frame, const Array3 & linear,
    const Covariance6 & covariance) {
      new (self) TwistSample{stamp, frame, linear, covariance};
    },
    "stamp"_a = 0, "frame"_a = SourceFrame::BASE, "linear"_a = Array3{0.0, 0.0, 0.0},
    "covariance"_a = Covariance6{})
  .def_rw("stamp", &TwistSample::stamp)
  .def_rw("frame", &TwistSample::frame)
  .def_rw("linear", &TwistSample::linear)
  .def_rw("covariance", &TwistSample::covariance);

  nb::class_<TwistInBase>(m, "TwistInBase", "The vehicle's velocity in its own frame.")
  .def_prop_ro("linear", [](const TwistInBase & twist) {return toArray(twist.linear);})
  .def_prop_ro("angular", [](const TwistInBase & twist) {return toArray(twist.angular);});

  nb::class_<Outputs>(
    m, "Outputs",
    "What the filter produces. The tree to publish is earth_to_map, published_map_to_odom and "
    "odom_to_base; map_to_odom and internal_twist_in_base are the same before the output "
    "smoothing.")
  .def_ro("earth_to_map", &Outputs::earth_to_map)
  .def_ro("map_to_odom", &Outputs::map_to_odom)
  .def_ro("published_map_to_odom", &Outputs::published_map_to_odom)
  .def_ro("odom_to_base", &Outputs::odom_to_base)
  .def_ro("twist_in_base", &Outputs::twist_in_base)
  .def_ro("internal_twist_in_base", &Outputs::internal_twist_in_base)
  .def_prop_ro(
    "earth_to_base",
    [](const Outputs & outputs) {
      return outputs.earth_to_map * outputs.published_map_to_odom * outputs.odom_to_base;
    },
    "The vehicle's pose in the earth frame, as the published tree composes it.");

  nb::class_<Filter>(
    m, "Filter",
    "The simple_ekf filter. It reads no clock: every time is an argument, in nanoseconds.")
  .def(
    "__init__",
    [](Filter * self, const Config & config) {new (self) Filter(config, logToPython);},
    "config"_a = Config(),
    "It logs to the Python logger \"simple_ekf_core\", configured as any other: "
    "logging.basicConfig(level=logging.INFO) shows the INFO lines as well.")
  .def("add_source", &Filter::addSource, "config"_a, "Register a source; returns its id.")
  .def_prop_ro(
    "config", [](const Filter & filter) {return filter.config();},
    "The configuration in use, after validation.")
  .def_prop_ro(
    "outputs", [](const Filter & filter) {return filter.outputs();},
    "A copy of the current outputs.")
  .def_prop_ro(
    "state",
    [](const Filter & filter) {
      const auto & data = filter.state().data;
      return toNumpy(std::vector<double>(data.begin(), data.end()), {data.size()});
    },
    nb::rv_policy::automatic,
    "The EKF state: x, y, z, vx, vy, vz, roll, pitch, yaw, then the accelerometer and "
    "gyroscope biases (STATE_NAMES).")
  .def_prop_ro(
    "state_covariance",
    [](const Filter & filter) {
      const auto & data = filter.stateCovariance().data;
      return toNumpy(std::vector<double>(data.begin(), data.end()), {15, 15});
    },
    nb::rv_policy::automatic, "The 15x15 state covariance.")
  .def("is_earth_to_map_set", &Filter::isEarthToMapSet)
  .def("mark_earth_to_map_set", &Filter::markEarthToMapSet)
  .def("set_earth_to_map", &Filter::setEarthToMap, "earth_to_map"_a)
  .def(
    "set_earth_to_map_from_first_pose", &Filter::setEarthToMapFromFirstPose, "pose"_a,
    "frame"_a)
  .def("set_offboard", &Filter::setOffboard, "offboard"_a)
  .def(
    "should_throttle_update", &Filter::shouldThrottleUpdate, "source"_a, "stamp"_a,
    "Whether a measurement comes too soon to respect the source's update_rate_hz.")
  .def(
    "is_repeated_position",
    [](Filter & filter, SourceId source, const Array3 & position, Nanoseconds now) {
      return filter.isRepeatedPosition(source, toVector(position), now);
    },
    "source"_a, "position"_a, "now"_a,
    "Whether a measurement repeats the source's last position, when it rejects repeats.")
  .def("on_imu", &Filter::onImu, "imu"_a)
  .def("on_pose", &Filter::onPose, "source"_a, "pose"_a, "now"_a)
  .def("on_twist", &Filter::onTwist, "source"_a, "twist"_a, "now"_a)
  .def(
    "on_tick", &Filter::onTick, "now"_a,
    "Advance the output smoothing and the pre-flight correction; true if it corrected.");

  m.def(
    "generate_covariance_from_config", &simple_ekf_core::generateCovarianceFromConfig,
    "config"_a, "The 6x6 covariance a source configured with fixed variances is believed with.");
  m.def(
    "get_covariance_with_config", &simple_ekf_core::getCovarianceWithConfig,
    "covariance"_a, "config"_a,
    "A message's pose covariance, with the source's configured values applied.");
  m.def(
    "get_linear_covariance_with_config", &simple_ekf_core::getLinearCovarianceWithConfig,
    "covariance"_a, "config"_a,
    "A message's twist covariance, with the source's configured values applied.");
}
